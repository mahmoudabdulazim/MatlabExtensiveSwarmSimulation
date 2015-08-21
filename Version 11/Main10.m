clear, close all; clc, rng(3);
% =============================
% Map parameters:
% ===============
L = 25; % Map_length 
W = 25; % Map_width
H = 3;  % Map_height
Sector_size = 5; Cell_size = 1;
sector_max = 1;
RSSI_range = 100; %communications prameter
% ================
% Wall parameters:
% ================
T = .1; % Wall_thickness

Walls_xlimit = ...
    [0 W/2-1;
     W/2+1 W;
     -T T;
     W-T W+T;
     0 W];
 
Walls_ylimit = ...
    [-T T;
     -T T;
     0 L;
     0 L;
     L-T L+T];
 
Walls_zlimit = ...
    [0 H;
    0 H;
    0 H;
    0 H;
    0 H];

Walls_transparency = [0.1; 0.1; 0.1; 0.75; 0.75];

% ==================
% Virtual boundries: for setting random targets
% ==================

min_Vertical = [-inf];
min_Horizontal = [0]; % at y = 0

% ====================
% Charging parameters:
% ====================

charge = 0;
Charging_stations_x = [2; L-2];
Charging_stations_y = [2; W-2];

% =================
% Swarm parameters:
% =================
Swarm_size = 1;
HR = 0; % RI_flying_height

% RI initial positions:
Px_ini = W/2; Py_ini = -2;

% RI arrival times:
lambda = 10; % seconds
inter_arrival_ts = poissrnd(lambda, 1, Swarm_size);
arrival_ts = cumsum(inter_arrival_ts);

arrival_ts = [1:2:9];

% ================
% Task parameters:
% ================

% Tasks = {'Explore' 'Search'};
% Skills = {'Lead' 'Follow' 'Replace Leader' 'Allign' 'Gather' 'Disperse' 'Charge' 'Aid' 'Map' 'Drag' 'Shape'};

Task_ini = 'Traverse'; % 0 for low or 1 for high
Targetx_ini = W/2; Targety_ini = L/2;

% 'Lead' parameters:
% - Become leader (reasons?)
% - Broadcast leader-tag, position, velocity, distance from leader (function of speed)
% - Broadcast follower(s)
% 'Follow' parameters:
% - Locate leader
% - Catch up with leader (max velocity) if distance greater than threshold
% - Reserve follower position (in a multi-follower shape)
% - Constantly monitor & match leader path & speed
% 'Replace Leader' parameters:
% - Calculate appropriate replacement (distributed algorithm)
% - Replace leader
%------ Samy Amin
% 'Location feedback (by sector)'
% - Vector and speed
% - Only inform those who are in the sector
% 'Map Pixilation'
% - Secotorization of the map (with tags)
% - Only invit in the same sector if further exploration is needed for
% another sector (easy monitoring and controled rate of collision )

% Task flow: search -> explore, explore -> discover, discover -> explore,
% explore p1 -> explore p2

% ===============
% Initialization:
% ===============

deltaT = 0.05;
arrival_ts = arrival_ts(1:Swarm_size);


h1 = []; h2 = []; h4 = []; h5 = [];
ri = [];
% Map.Sensed = []; Map_index = 0;
Map.sect_size = Sector_size; Map.cell_size = Cell_size;
Map.sect_max = sector_max;

Map.min_ver = min_Vertical; Map.min_hor = min_Horizontal; % at y = 0

Map.RSSI_range = RSSI_range;
Map.L = L; Map.W = W; % for annotation purposes

Map.reg_sectors = []; Map.reg_sectors_ind = 1;

Targetxy_ini = [Targetx_ini; Targety_ini];

Walls_x = [Walls_xlimit(:,1) Walls_xlimit(:,1) Walls_xlimit(:,2) Walls_xlimit(:,2)];
Walls_y = [Walls_ylimit(:,1) Walls_ylimit(:,2) Walls_ylimit(:,2) Walls_ylimit(:,1)];
Walls_z = [Walls_zlimit(:,1) Walls_zlimit(:,1) Walls_zlimit(:,2) Walls_zlimit(:,2)];

Walls_xbox = [Walls_x Walls_x(:,1)];
Walls_ybox = [Walls_y Walls_y(:,1)];

Map.Sensor.Walls_poly = [];
for wall = 1:size(Walls_x,1)
    Map.Sensor.Walls_poly = [Map.Sensor.Walls_poly, [Walls_xbox(wall,:), nan; Walls_ybox(wall,:), nan]];
end

Map.Walls_xbox = Walls_xbox;
Map.Walls_ybox = Walls_ybox;

Map.Walls = []; % discovered walls (each row is a wall [Px_start Py_start Px_end Py_end])

% =============
% Initial Plot:
% =============
    % set(gca, 'nextplot','replacechildren', 'Visible','off');
    % M(1:400) = struct('cdata',[], 'colormap',[]);    
    x = get(0, 'screensize'); hfig = figure; set(hfig, 'position', [10 55 3*x(3)/4 x(4)-150])%,'NextPlot','replacechildren');
    %--------------
    subplot(3,2,[1,3]), hold on, title('2D Layout')
    %--------------
    if charge == 1, plot(Charging_stations_x, Charging_stations_y, 'o'), end % plot charging stations
    for wall=1:size(Walls_x,1), fill(Walls_x(wall,:),Walls_y(wall,:),'r'), end % plot walls
    axis([-1 L+1 -1 W+1]), xlabel('x'), ylabel('y'), grid on,
    %--------------
    subplot(3,2,[2,4]); hold on, title('2D Reconstructed Map')
    %--------------
%     if charge == 1, plot(Charging_stations_x, Charging_stations_y, 'o'), end % plot charging stations
%     for wall=1:size(Walls_x,1), fill(Walls_x(wall,:),Walls_y(wall,:),'r'), end % plot walls
    fill([-1 -1 W+1 W+1],[-1 L+1 L+1 -1],[0.5 0.5 0.5],'FaceAlpha',1)
    axis([-1 L+1 -1 W+1]), xlabel('x'), ylabel('y'), grid on,
    %--------------
    subplot(3,2,5),  
    %--------------
    if charge == 1, plot3(Charging_stations_x, Charging_stations_y, [2,2], 'o'), else plot3(0,0,0,'Marker','none'), end
    hold on,
    for wall=1:size(Walls_x,1), patch(Walls_xlimit(wall,[1,2,2,1]),Walls_ylimit(wall,[1,2,2,1]),Walls_zlimit(wall,[1,1,2,2]),'r', 'FaceAlpha', Walls_transparency(wall)), end
    axis([-1 L+1 -1 W+1 0 H]), xlabel('x'), ylabel('y'), zlabel('z'), grid on,
    title('3D Layout')
    %--------------
%     subplot(3,2,6), axis off,
    %--------------
%     subplot_info = get(gca,'Position');
%     str1 = {...
%         ['RI/Mode:'],
%         ['Px, Py: '],
%         ['Target: '],
%         ['Vx, Vy: '],
%         ['Theta : '],
%         ['Sensors:']};
%     str1_plot_info = [subplot_info(1), subplot_info(2), subplot_info(3)/6, 6*subplot_info(4)/8];
%     annotation('textbox','Position',str1_plot_info,'VerticalAlignment','middle','String',str1)
 
for i = 1/deltaT : 360/deltaT %60*5*30       % time instants in 30frame/seconds

    %New Arrivals
    x = find(arrival_ts<=i*deltaT);
    if ~isempty(x)
        New_RIs = length(x);
        for x = 1:New_RIs
            New_RI = RI(Px_ini, Py_ini, Task_ini, Targetxy_ini, Map);
            Map = New_RI.Map; % New_RI = rmfield(New_RI,'Map');
            ri = [ri New_RI];
        end
        arrival_ts = setxor(arrival_ts,arrival_ts(x));
    end
    
    % Update positions
    if(~isempty(ri)), [ri, Map] = update_RI(ri,deltaT,Map); end
% Map, pause

    %========
    %PLOTTING
    %========
    if( ~isempty(ri) )    
%         %--------------
%         subplot(3,2,[1,3]),
%         %--------------
% %         delete(h1);
%         delete(h1);        
% %             for x = 1:length(ri), h1(x) = rectangle('Position',[[[ri(x).Px]-[ri(x).sensing_distance]],[[ri(x).Py]-[ri(x).sensing_distance]],[2*[ri(x).sensing_distance]],[2*[ri(x).sensing_distance]]],'Curvature',[1,1],'FaceColor','y','EdgeColor','y'); end % plot sensor distance
%             h1 = fill([ri(:).XVertices], [ri(:).YVertices],'b'); % plot RIs
%         %--------------
%         subplot(3,2,5),
%         %--------------
%         delete(h4);delete(h5);
% %             delete(h3); for x = 1:length(ri), h3 = rectangle('Position',[[[ri(x).Px]-[ri(x).sensing_distance]],[[ri(x).Py]-[ri(x).sensing_distance]],[2*[ri(x).sensing_distance]],[2*[ri(x).sensing_distance]]],'Curvature',[1,1],'FaceColor','y','EdgeColor','y'); end % plot sensor distance
%             h4 = fill3([ri(:).XVertices], [ri(:).YVertices], HR*ones(size([ri(:).YVertices])),'b'); % plot RIs
%             h5 = stem3([ri(:).Px], [ri(:).Py], HR*ones(size([ri(:).Py])), ':b','Marker','none');
%         pause(0.00001)
    end
    
%     if( ~isempty(collided_cars) ), plot(collided_cars(:,1), collided_cars(:,2),'or'); end  
%     axis equal; hold off;
%     M(i) = getframe(gca);  

end 
% % % % movie2avi(M, 'myPeaks.avi', 'compression', 'none');