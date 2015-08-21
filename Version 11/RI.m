classdef RI < handle
    
properties 
    Px;
    Py;
    Pz = 0;
    PX; %Group center
    PY; %Group center
    Vx;
    Vy;
    Fit_value
    Pause_flag = 0;
    XVertices;
    YVertices;
    V_max = 2; % m/s
    V_min = 0; % m/s

    RI_width = 0.5;
    RI_length = 0.5;
    Type;
    RI_handle_plot
    RI_handle_text

    sensing_distance = [1.4 1.4 1.4 1.4 1.4]; % meter
    sensor_threshold = [1.5 0.7 0.7 0.5 0.5]; % meter
    sensing_angles = [0 45 -45 90 -90]; % from front
    sensing_used = 5;
    Sensor_status;
    Sensor_handles;

    Current_cell;
    Current_sector;
    Map;
    Sensor_update; % 'none', 'frequent', 'non_frequent'
    Map_sensed;
    Map_not_sensed;

    Color;
    Task =[];
    Task_spec = [];
%     Task_velocity;
    Task_parameters;
    Task_xy =[]; %eventual target
    Target_xy; %temp target (can be used in setting courses that are not straight)
    target_disp
    
end

methods 
    function obj = RI(Px_ini, Py_ini, Task_ini, Targetxy_ini, Map)

        %Initial Task
%         obj.Task = []; obj.Task_xy = []; % old task and task termination position
        obj = set_task(obj, [], Task_ini, Targetxy_ini, Map, 0, 'initialization');
        obj.target_disp = obj.Target_xy; 
        %Set initial position
        obj.Px = Px_ini;
        obj.Py = Py_ini;
        
        %Initial angle & velocity from initial position & target
        DeltaX = obj.Target_xy(1) - Px_ini;  
        DeltaY = obj.Target_xy(2) - Py_ini;
        
        theta = atan2d(DeltaY,DeltaX);

        obj.Vx = obj.Task_parameters.V_task*cosd(theta);
        obj.Vy = obj.Task_parameters.V_task*sind(theta);
        
        %Calculating vertices & group center from velocity & position
        obj = update_vertices(obj);

        %Initiating sensors:
        obj.Sensor_status = zeros(1,obj.sensing_used);

        %Initializing map parameters:
        [obj.Current_cell, obj.Current_sector] = grid_position(obj.Px, obj.Py, Map);

        old_sector = nan;
        Map = RI_count(old_sector, obj.Current_sector, Map);
        obj.Sensor_update = update_freq(obj.Current_sector, Map);
        Map = map_cell(obj.Current_sector, obj.Current_cell, Map, 0);            

        obj.Map = Map;
        obj.Map.walls = [];
        obj.Map.discovered_walls = [];
        obj.Map.received_walls = [];
        obj.Map.walls_poly =[];

    end

    function [obj, Map] = update_RI(obj, deltaT, Map)
        disp_modules = 0;
        if disp_modules == 1, disp('update_RI: initialization...'), end
        
        % Update position according to velocity:
        X = num2cell([obj.Px] + [obj.Vx].*~[obj.Pause_flag]*deltaT);   [obj(:).Px] = X{:};
        Y = num2cell([obj.Py] + [obj.Vy].*~[obj.Pause_flag]*deltaT);   [obj(:).Py] = Y{:};
        
        % Update vertices according to position:
        obj = update_vertices(obj);
        plot_RI([obj(:).XVertices], [obj(:).YVertices], [obj(:).Color], [obj(:).Px], [obj(:).Py], [obj(:).Pz]);
        
%         for o = 1:length(obj)
%             obj(o).RI_handle_plot = plot_RI(obj(o).RI_handle_plot, obj(o).XVertices, obj(o).YVertices, obj(o).Color);
%         end
        
        % Update task according to map position and sensor status (for each RI):
    for o = 1:length(obj)
%         constructed_string = construct_str('display',o,obj(o),[]);
%         obj(o).RI_handle_text = annotate_update(obj(o).RI_handle_text, constructed_string,'display', o);
        
        %% ================
        % SENSOR INTERUPTS: (Check sensors)
        % =================
        if disp_modules == 1, disp('update_RI: sensor check...'), end
        
        prev_Sensor_status = obj(o).Sensor_status; 
        [obj(o), object_distances, points] = sensor_RI(obj(o), Map.Sensor, [obj(1:end~=o).Px], [obj(1:end~=o).Py], [obj(1:end~=o).XVertices], [obj(1:end~=o).YVertices]);
        % -------------
%         if any(obj(o).Sensor_status == 1), disp('------ Sensor Interruption ------'), disp(['Sensor status : ', mat2str(obj(o).Sensor_status)]), disp(['Sensed distances : ', mat2str(round(object_distances,2))]), disp('---------------------------------'), end
        % ======
        % EVENT: front sensor detected wall
        % ======
        wall_ended=0; %initialization for wall_ended flag used to quit collision while tracking 
        cond1_01 = (obj(o).Sensor_status(1,1) == 1) && (prev_Sensor_status(1,1) == 0); 
        if cond1_01
            
%             [wall_discovered, wall_being_discovered, cell_status] = is_wall_discovered(points(:,obj(o).Sensor_status(1:3)==1),Map);

            [wall_discovered, wall_being_discovered, cell_status] = is_wall_discovered(points(:,1),Map);

            if  (~wall_discovered && ~wall_being_discovered) || (wall_being_discovered && (imag(cell_status)-1000 == o))% && (strcmp(obj(o).Task, 'Explore P0')||strcmp(obj(o).Task, 'Explore P1'))
                obj(o).Task_parameters.discover_wall = 1;
            else %if wall is discovered
                obj(o).Task_parameters.discover_wall = 0;
                if ~wall_being_discovered
                    % is wall discovered or revieved by this RI?
                    is_wall_known = ismember(cell_status(1), [obj(o).Map.discovered_walls obj(o).Map.received_walls]);
                    if ~is_wall_known %if not, download wall info
                        obj(o).Map.walls_poly = [obj(o).Map.walls_poly, [Map.Walls(cell_status(1),[1,2])]', [Map.Walls(cell_status(1),[3,4])]', [nan; nan]];
                        obj(o).Map.received_walls = [obj(o).Map.received_walls, cell_status(1)];
                    end
                end
            end
%         points(:,1)
%         [cell, sector] = grid_position(points(1,1), points(2,1), Map)
        
        end
        
        discover_wall = obj(o).Task_parameters.discover_wall;
%         if cond1_01,pause,end
        % ======
        % EVENT: front sensor detected/or detecting RI
        % ======
        cond1_02 = (obj(o).Sensor_status(1,1) == 2) && (prev_Sensor_status(1,1) == 0);
        cond1_2  = (obj(o).Sensor_status(1,1) == 2);
        cond1_20 = (obj(o).Sensor_status(1,1) == 0) && (prev_Sensor_status(1,1) == 2);
%         if cond1_02 || cond1_2
%             
%             % ID the detected RI
%             [detected_RI_cell, detected_RI_sector] = grid_position(points(1,1), points(2,1), Map);
%             detected_RI_ID = read_map(detected_RI_sector, detected_RI_cell, Map)
%             RI_is_in_track_mode = strcmp(obj(o).Task,'Track');
%             
%             if RI_is_in_track_mode
%                 
%                 is_tracking_right_wall = strcmp(obj(o).Task_parameters.track,'right');
%                 is_tracking_left_wall = strcmp(obj(o).Task_parameters.track,'left');
%                 
%                 fprintf('Detected RI ID = %d ',detected_RI_ID)
%                 pause
%                 if is_tracking_right_wall,       wall_point = points(:,4);
%                 elseif is_tracking_left_wall,    wall_point = points(:,5);
%                 end
%                 
%                 [current_wall_discovered, current_wall_being_discovered, current_cell_status] = is_wall_discovered(fourth_point([obj(o).Px;obj(o).Py],wall_point,points(:,1)),Map);
%                 if (current_wall_discovered || current_wall_being_discovered) && obj(o).Pause_flag && (o > detected_RI_ID -1000)
%                    obj(o).Pause_flag = 0;
%                    wall_ended=1;
%                 end
%                     
%             end
%         end
        
        % ======
        % EVENT: front and 45 sensor detected wall
        % ======
        % ----------------------------------------------------------
        %% ==============
        % MAP INTERUPTS: (Check map)
        % ==============
        if disp_modules == 1, disp('update_RI: map check...'), end
        
        % Calculate current sector/cell:
        old_cell = obj(o).Current_cell;   old_sector = obj(o).Current_sector;
        [obj(o).Current_cell, obj(o).Current_sector] = grid_position(obj(o).Px, obj(o).Py, Map);
        %  ---------------------------------------------------------
        % ======
        % EVENT: cell has changed
        % ======
        cell_changed = any(old_cell ~= obj(o).Current_cell); sector_changed = 0;
        if cell_changed % Map_update with current position (optional: RI count, cell_explored, sect_explored)     
            
            % Map old cell as empty space  and map new one with RI_ID:
            Map = map_cell(old_sector, old_cell, Map, 0);
            Map = map_cell(obj(o).Current_sector, obj(o).Current_cell, Map, 1000+o);
            cell_status = read_map(obj(o).Current_sector, obj(o).Current_cell, Map);
            if (o==1)  || (o==4)
                fprintf('Object is Object no: %i ',o)
                fprintf('Current Cell is: %s & Current Sector is: %s \n',mat2str(obj(o).Current_cell),obj(o).Current_sector)
                fprintf('Cell status is: %f , object no: %i \n',cell_status,o)
                fprintf('\n \n \n')
            end
            % Check if sector has changed:
            sector_changed = ~strcmp(old_sector, obj(o).Current_sector);
        
            % ======
            % EVENT: sector has changed
            % ======
            if sector_changed

                % Update old non-frequent updates at sector change:
                if strcmp(obj(o).Sensor_update,'non-frequent') && ~isempty(obj(o).Map_sensed)
                    disp('Updating map at sector change for NON-FREQUENT UPDATING RI')
                    Map = update_map(old_sector, Map, obj(o).Map_sensed, obj(o).Map_not_sensed);
                    obj(o).Map_sensed = []; obj(o).Map_not_sensed = [];
                end
   
                % Update old and new sector RI count:
                Map = RI_count(old_sector, obj(o).Current_sector, Map);

                % Check new update status & if sector is overloaded:
                % ======
                % EVENT: sector overloaded --> pause motion
                % ======
                [obj(o).Sensor_update, obj(o).Pause_flag] = update_freq(obj(o).Current_sector, Map);
                %the update_freq function needs to be provoked not when
                %the RI changes sectors, but when any RI changes sectors                
%             else, sector_overloaded = 0;
            end
%         else, sector_overloaded =
%         is_sector_overloaded(obj(o).Current_sector,Map); end
        else
            % ======
            % EVENT: RI is paused and sector is no longer overloaded (cell has not changed)
            % ======
            sector_overloaded = eval(['Map.sect_', obj(o).Current_sector,'.RIs > Map.sect_max']);
            if obj(o).Pause_flag && ~sector_overloaded,    obj(o).Pause_flag = 0; end
        end
        
        % PAUSE FLAG:
        obj(o).Pause_flag = obj(o).Pause_flag || cond1_02 || cond1_2 || sum(obj(o).Sensor_status(1:3)==1)>3;
            if obj(o).Pause_flag && cond1_20
                obj(o).Pause_flag = 0;
            end
        if cond1_02 || cond1_2
            
            % ID the detected RI
            [detected_RI_cell, detected_RI_sector] = grid_position(points(1,1), points(2,1), Map);
            detected_RI_ID = read_map(detected_RI_sector, detected_RI_cell, Map)
            RI_is_in_track_mode = strcmp(obj(o).Task,'Track');
            
            if RI_is_in_track_mode
                
                is_tracking_right_wall = strcmp(obj(o).Task_parameters.track,'right');
                is_tracking_left_wall = strcmp(obj(o).Task_parameters.track,'left');
                
                fprintf('Detected RI ID = %f \n',detected_RI_ID)
                if is_tracking_right_wall,       wall_point = points(:,4);
                elseif is_tracking_left_wall,    wall_point = points(:,5);
                end
                FOURTH  =fourth_point([obj(o).Px;obj(o).Py],wall_point,points(:,1));
                if (o==1) || (o==4)
                    [current_wall_discovered, current_wall_being_discovered, current_cell_status] = is_wall_discovered(FOURTH,Map);
                    fprintf('Current Object is Object no: %i ',o)
                    fprintf('Fourth point is: %f ',FOURTH)
                    [FOURTH_Cell, FOURTH_Sect]=grid_position(FOURTH(1),FOURTH(2),Map);
                    fprintf('Fourth Point Sector is: %s and Cell is : %s \n',FOURTH_Sect,mat2str(FOURTH_Cell))
                    fprintf('Flags are: Discovered = %f & Being_Discovered = %f \n',current_wall_discovered,current_wall_being_discovered)
                    fprintf('Current Cell Status is: %s \n',mat2str(current_cell_status))
                    fprintf(' \n \n \n ')
                end
                if (current_wall_discovered || current_wall_being_discovered) && obj(o).Pause_flag && (o > detected_RI_ID -1000)
                   obj(o).Pause_flag = 0;
                   wall_ended=1;
                end
                pause
                    
            end
        end
        %% Update Map with sensor readings (detected vacacncies/obstacles if undiscovered):
        if disp_modules == 1, disp('update_RI: map updating...'), end
        
        if ~strcmp(obj(o).Sensor_update,'none') && (any(any(~isnan(points))))% && cell_changed

            switch obj(o).Sensor_update
                case 'frequent'
                    Map = update_map(obj(o).Current_sector, Map, points(:,obj(o).Sensor_status==1), points(:,obj(o).Sensor_status==0));
                
                case 'non-frequent'
                    obj(o).Map_sensed = points(:,obj(o).Sensor_status==1); 
                    obj(o).Map_not_sensed = points(:,obj(o).Sensor_status==0);
            end
            
            if strcmp(obj(o).Task, 'Track') && cell_changed %&& ~obj(o).Task_parameters.following_up
                
                if discover_wall
                    
                    is_tracking_right_wall = strcmp(obj(o).Task_parameters.track,'right');
                    is_tracking_left_wall = strcmp(obj(o).Task_parameters.track,'left');

                    % Not sure why this is for:
                    if is_tracking_right_wall
                        wall_point = points(:,4);
                    elseif is_tracking_left_wall
                        wall_point = points(:,5);
                    end
                    
                    [wall_cell, wall_sect] = grid_position(wall_point(1,1), wall_point(2,1), Map);
                    [wall_discovered, wall_being_discovered, cell_status] = is_wall_discovered(wall_point, Map);
                    
                    if  (~wall_discovered && ~wall_being_discovered) || (wall_being_discovered && (imag(cell_status)-1000 == o))% && (strcmp(obj(o).Task, 'Explore P0')||strcmp(obj(o).Task, 'Explore P1'))
                        obj(o).Task_parameters.discover_wall = 1;
                    else %if wall is discovered
                        obj(o).Task_parameters.discover_wall = 0;
                        if ~wall_being_discovered
                            % is wall discovered or revieved by this RI?
                            is_wall_known = ismember(cell_status(1), [obj(o).Map.discovered_walls obj(o).Map.received_walls]);
                            if ~is_wall_known %if not, download wall info
                                obj(o).Map.walls_poly = [obj(o).Map.walls_poly, [Map.Walls(cell_status(1),[1,2])]', [Map.Walls(cell_status(1),[3,4])]', [nan; nan]];
                                obj(o).Map.received_walls = [obj(o).Map.received_walls, cell_status(1)];
                            end
                        end
                    end

                
%                 [cell, sector] = grid_position(points(1,1), points(2,1), Map)
%                 discover_wall = obj(o).Task_parameters.discover_wall;
%                 pause

                    Map = map_cell(wall_sect, wall_cell, Map, 0.75 + 1i*(o+1000));
                    plot_cell(wall_sect, wall_cell, Map, 0.75);
%                     wall_cell
%                     wall_sect
%                     pause

    %                 if cell_changed
                        obj(o).Task_parameters.tracked_cells = [obj(o).Task_parameters.tracked_cells; wall_cell];
                        obj(o).Task_parameters.tracked_sects = [obj(o).Task_parameters.tracked_sects; wall_sect];
    %                 end
                else
                    % communicate or consider it wall_ended event
                end
            end
        end
        %% ----------------------------------------------------------------
        if disp_modules == 1, disp('update_RI: task logic...'), end
        
        target_reached = all(abs([obj(o).Px;obj(o).Py] - obj(o).Task_xy) < [0.2 ; 0.2]);
        sector_explored = eval(['Map.sect_',obj(o).Current_sector,'.explored']);
        off_map = (obj(o).Px - obj(o).PX)^2 + (obj(o).Py - obj(o).PY)^2 > Map.RSSI_range^2;  % r_max should be a variable f(anchors)

%         isequal(prev_Sensor_status,obj(o).Sensor_status)
%         prev_Sensor_status
%         curr_Sensor_status = obj(o).Sensor_status
        %%----------------- 
        % Task Transitions:
        %------------------
        % If Sensor status has changed or distance from object is below threshold:
        if ~isequal(prev_Sensor_status,obj(o).Sensor_status) ...
                || any(object_distances <= obj(o).sensor_threshold) ...
                || target_reached || off_map || sector_explored

            % -------------
            % Sensor cases:
            % -------------
% target_reached
            % Front sensor detected nothing:
            cond1_0 = (obj(o).Sensor_status(1,1) == 0);

            % Track wall event:
            track_wall = sum(obj(o).Sensor_status(1:3)==1) > 1 && obj(o).Task_parameters.discover_wall;
            
            % Side wall has ended:
            cond4_10 = (obj(o).Sensor_status(1,4) == 0) && (prev_Sensor_status(1,4) == 1);
            cond5_10 = (obj(o).Sensor_status(1,5) == 0) && (prev_Sensor_status(1,5) == 1);

% %         task_parameters = obj(o).Task_parameters
% % %         
% %         isfield(obj(o).Task_parameters,'Parent_task')
% %         if isfield(obj(o).Task_parameters,'Parent_task')
% %           parent_task_parameters = obj(o).Task_parameters.Parent_task_parameters
% %         end
        if isfield(obj(o).Task_parameters,'Parent_task') 
            if eval(obj(o).Task_parameters.Parent_task_parameters.switch_cond)
                post_switch = obj(o).Task_parameters.Parent_task_parameters.post_switch
                
                P_Task_par = obj(o).Task_parameters.Parent_task_parameters
                % --------------------------------------
                obj(o) = set_task(obj(o), o, [], [], Map, -1, 'of switch to parent-task condition');
                % --------------------------------------
                eval(post_switch)
                Task_par = obj(o).Task_parameters
                
                % display
                display_parent_tasks(obj(o))
                            
                task_target = obj(o).Task_xy
                target = obj(o).Target_xy
                disp('320'),pause(1)
            end
              
        else
                
            switch obj(o).Task
                                
                case 'Traverse' % responds to 0-sensor by finding a path (from map/discovery)  
                
                    approaching = obj(o).Task_parameters.approaching;
                    
                    % Traverse --------> Explore
                    if target_reached
                        % --------------------------------------
                        obj(o) = set_task(obj(o), o, 'Explore', 'random', Map, 0, 'target was reached');
                        % --------------------------------------

                    % Traverse --------> Anchor
                    elseif (approaching == 1 && off_map == 1)
                        % --------------------------------------
                        obj(o) = set_task(obj(o), o, 'Anchor', [], Map, 1, 'of going off map while approaching');
                        % --------------------------------------
 
                    % Traverse --------> Track (if obstacle is not discovered)
                    elseif track_wall % sum(obj(o).Sensor_status(1:3)==1) > 1 && obj(o).Task_parameters.discover_wall
                        
                        % if map not discovered, discover a route by setting target to move perpendicular to sensed object (tracking it) and recording personal best and decision taken
                        obj(o).Task_spec = 'decide';
                        % --------------------------------------
                        obj(o) = set_task(obj(o), o, 'Track', points(:,obj(o).Sensor_status(1:3)==1), Map, 1, 'of tracking wall to find route');
                        % --------------------------------------      
                        [obj(o), Map] = map_wall(obj(o), o, points(:,1), Map, 'midpoint');
                  
                    elseif cond1_01 && ~obj(o).Task_parameters.discover_wall % if obstacle is registered
                        
                        if wall_being_discovered
%                             pause
                        else % Continue in EP1 by using map
                            set_new_course % by setting target to eastmost or westmost path (whichever's closest) while recording personal best and decision
                        end
                    end
                % =========================================================    
                case 'Explore' % responds by 0-sensor by tracking wall through 90-sensor

                    % EP0 --------> Track (Two sensors triggered)
                    if track_wall % sum(obj(o).Sensor_status(1:3)==1) > 1 && obj(o).Task_parameters.discover_wall
                        
                        obj(o).Task_spec = 'decide';
                        % --------------------------------------
                        [obj(o), Map] = set_task(obj(o), o, 'Track', points(:,obj(o).Sensor_status(1:3)==1), Map, 0, 'of a track wall event');
                        % --------------------------------------
                        [obj(o), Map] = map_wall(obj(o), o, points(:,1), Map, 'midpoint');
    
                    % EP0 --------> EP0
                    elseif off_map || target_reached || (cond1_01 && ~obj(o).Task_parameters.discover_wall)

                        % --------------------------------------
                        obj(o) = set_task(obj(o), o, 'Explore', 'random', Map, 0, 'target was reached, going off map, or neglecting wall');
                        % --------------------------------------
    
                    end
                % =========================================================    
                case 'Track' % tracking wall and keep tracking new ones
                    % responds to 0 sensor detections & 90 distance decrease below threshold by correcting course
                    % responds to 90 sensor change by returning to explore task(with new random target and 0 priority)

%                     is_wall_already_discovered = ...
%                         obj(o).Task_parameters.discovered_count >= obj(o).Task_parameters.discovered_max;
% % %                     following_up = obj(o).Task_parameters.following_up;
                    discover_wall = obj(o).Task_parameters.discover_wall;

                    is_tracking_right_wall = strcmp(obj(o).Task_parameters.track,'right');
                    is_tracking_left_wall = strcmp(obj(o).Task_parameters.track,'left');
                    
                    wall_ended = (cond4_10&&is_tracking_right_wall) || (cond5_10 && is_tracking_left_wall) || wall_ended;
                    new_wall = (is_tracking_right_wall&&obj(o).Sensor_status(3)) || (is_tracking_left_wall && obj(o).Sensor_status(2));
                    
                    % -----------------------------------------------------
                    if off_map || wall_ended || new_wall ||  ...
                            (sector_changed||sector_explored) || ~discover_wall
                    % ( no new walls && wall has ended  ) or ( sector/wall has been already explored )

                        decision_point = isfield(obj(o).Task_parameters,'decision_point');
                        
                        % --------------- MAP WALLS
                        if is_tracking_right_wall,     wall_point = points(:,4);
                        elseif is_tracking_left_wall,  wall_point = points(:,5);
                        end

                        if wall_ended,     wall_end = wall_point;
                        elseif new_wall,   wall_end = fourth_point([obj(o).Px; obj(o).Py], wall_point,  points(:,1));
                        else wall_end = fourth_point([obj(o).Px; obj(o).Py], wall_point,  points(:,1));
                        end
                            
                        % Track --> Track (follow_up)
                        if decision_point
%                             [obj(o), Map] = map_wall(obj(o), o, wall_point, Map, 'midpoint');
                            [obj(o), Map] = map_wall(obj(o), o, wall_end, Map, 'start of midpoint');
                            
                            % display
                            display_parent_tasks(obj(o));
                            
                            % continue in DP1 by following up on decision point
                            % --------------------------------------
                            obj(o).Task_parameters.switch_cond = 'sum(obj(o).Sensor_status(1:3)) > 1';%'all(abs([obj(o).Px;obj(o).Py] - obj(o).Task_parameters.Parent_task_parameters.decision_point) < [0.2 ; 0.2])';
                            obj(o).Task_parameters.post_switch = 'obj(o).Task_spec = ''follow up'';[obj(o), Map] = set_task(obj(o), o, ''Track'', points(:,obj(o).Sensor_status(1:3)==1), Map, 0, ''it is following up on current wall'');';

                            obj(o) = set_task(obj(o), o, 'Traverse', obj(o).Task_parameters.decision_point, Map, 1, ' it is following up on current wall');
                            % --------------------------------------

                            % display
                            display_parent_tasks(obj(o));
                            
                            % --------------------------------------
                            obj(o).Task_parameters.switch_cond = 'target_reached';%'all(abs([obj(o).Px;obj(o).Py] - obj(o).Task_parameters.Parent_task_parameters.decision_point) < [0.2 ; 0.2])';
                            obj(o).Task_parameters.post_switch = 'theta = obj(o).Task_parameters.theta; obj(o).Target_xy = [NaN; NaN];';%'obj(o).task_spec = ''follow_up''';
                            
                            obj(o) = set_task(obj(o), o, 'Traverse', obj(o).Task_parameters.decision_point, Map, 1, 'of a decision point');
                            % --------------------------------------
                            
                            obj(o).Target_xy = obj(o).Task_parameters.decision_point;
                            obj(o).Task_xy = obj(o).Task_parameters.decision_point;
                            obj(o).Task_parameters.following_up = 1;

                            % display
                            display_parent_tasks(obj(o));
                            
                        else
                            
%                             [obj(o), Map] = map_wall(obj(o), o, wall_point, Map, 'midpoint');
                            
%                             if wall_ended,     wall_end = wall_point;
%                             elseif new_wall,   wall_end = wall_point; %+ ; !!!!!!!!!!!!!!!
%                             end
                            
                            [obj(o), Map] = map_wall(obj(o), o, wall_end, Map, 'end');
                            
%                             x = new_wall
%                             y = discover_wall
                            
                            % Track --> Track (new_wall)
                            if new_wall && discover_wall
                                    %  new wall detected on opposite 45 while tracking && cell not already explored

                                    obj(o).Task_spec = 'new wall'
                                    % --------------------------------------
                                    [obj(o), Map] = set_task(obj(o), o, 'Track', points(:,obj(o).Sensor_status(1:3)==1), Map,  0, 'of a new wall with no remaining decision point');
                                    % --------------------------------------
                                    
                                    [obj(o), Map] = map_wall(obj(o), o, [wall_end; points(:,1)], Map, 'start of new wall');
                            
                            % Track --> Explore/Traverse
                            else % wall_ended && no decision_point
                            
                                % switch to EP0
                                % --------------------------------------
                                obj(o) = set_task(obj(o), o, 'Explore', 'random', Map, 0, 'wall has ended with no remaining decision point');
                                % --------------------------------------
                            
                            end
                        end
                    end
                  
            end % switch

        end % if (parent_task)
        end % of sensing module
        
        if disp_modules == 1, disp('update_RI: set velocity according to target/task...'), end
        
        if size(obj(o).Target_xy,2)>1
            target_reached = all(abs([obj(o).Px;obj(o).Py] - obj(o).Target_xy(:,1)) < [0.3 ; 0.3]);
            if target_reached
                old_target = obj(o).Target_xy(:,1); 
                obj(o).Target_xy = obj(o).Target_xy(:,2:end);
                plot_target(obj(o).Target_xy(:,1), old_target, Map);
            end
        end
        
        if ~isequal(obj(o).Target_xy,obj(o).target_disp)
            disp(['Target_x = ', num2str(obj(o).Target_xy(1)), ', Target_y = ', num2str(obj(o).Target_xy(2))]), 
            obj(o).target_disp = obj(o).Target_xy;
        end

%         
        if ~any(isnan(obj(o).Target_xy))
            DeltaX = obj(o).Target_xy(1,1) - obj(o).Px;
            DeltaY = obj(o).Target_xy(2,1) - obj(o).Py;
            theta = atan2d(DeltaY,DeltaX);
        else
            if isempty(who('theta'))
                theta = atan2d(obj(o).Vy,obj(o).Vx);
            end
        end
            obj(o).Vx = obj(o).Task_parameters.V_task*cosd(theta);
            obj(o).Vy = obj(o).Task_parameters.V_task*sind(theta);
% % %             else
% % %                 obj(o).Vx = 0;
% % %                 obj(o).Vy = 0;
% % %             end
    end % object for-loop
%     disp('Pause Flags are: ')
%     obj.Pause_flag
    end
    
    function [obj, Map] = set_task(obj, o, task, parameter, Map, Queue, cause)
        
        Modes           = {'Explore', 'Traverse', 'Track', 'Anchor'};
        Mode_velocities = [obj.V_max, obj.V_max , obj.V_max/2, 0      ];
        Mode_colors     = {'b'      , 'r'       , 'y'       , 'k'     };
%         Modes_abr       = {'EP0'       , 'TP1'       , 'DP0};
        
        old_task = obj.Task;     old_target = obj.Task_xy;
        
    if Queue == -1 % Switch back to parent task
        
%         task_paramters = obj.Task_parameters
        obj.Task = obj.Task_parameters.Parent_task; 
        obj.Task_spec = obj.Task_parameters.Parent_task_spec;
        obj.Task_parameters = obj.Task_parameters.Parent_task_parameters;

        obj.Color = Mode_colors{strcmp(Modes,obj.Task)};
%         obj.Task_parameters = rmfield(obj.Task_parameters, {'Parent_task','Parent_task_parameters','Parent_task_spec'}); 

    else
        
        if Queue % Put parent task in the queue and switch to new task
            obj.Task_parameters.Parent_task_parameters = obj.Task_parameters;
            obj.Task_parameters.Parent_task = obj.Task;
            obj.Task_parameters.Parent_task_spec = obj.Task_spec;
        end
        
        obj.Task = task; 
        obj.Task_parameters.V_task = Mode_velocities(strcmp(Modes,task));
        obj.Color = Mode_colors{strcmp(Modes,task)};
        
        % General Task Parameters:
%         obj.Task_parameters.discovered_count = 0;
%         obj.Task_parameters.discovered_max = 2;
        
        switch task
            case 'Explore'
                
                % -------------------- Target
                [obj.Target_xy, ~] = ...
                        set_new_target(parameter,obj.Px, obj.Py, Map,obj.Map);
%                 obj.Target_xy = obj.Task_xy;
                % ---------------------------
                
                obj.Task_parameters.approaching = NaN;
                obj.Task_parameters.discover_wall = 0;
                                
            case 'Traverse'
                
                % -------------------- Target
                obj.Target_xy = parameter;
%                 obj.Target_xy = obj.Task_xy;
                % ---------------------------
                
                obj.Task_parameters.approaching = 1;
                obj.Task_parameters.discover_wall = 0;                

            case 'Track'

                theta = atan2d(obj.Vy,obj.Vx);
                
                switch obj.Task_spec
                
                    case 'decide'
                        
                        % -------------------- Target
                        if isfield(obj.Task_parameters, 'Parent_task') && strcmp(obj.Task_parameters.Parent_task, 'Traverse')
                            % if Parent_Task is not Traverse (else choose the
                            % direction closest to target
                        else
                            [obj.Target_xy, obj.Task_parameters.track] = ...
                                set_new_target('right or left',obj.Px, obj.Py, Map, ...
                                parameter, theta);
                        end
%                         obj.Target_xy = obj.Task_xy;
                        % ---------------------------
                        
                        obj.Task_parameters.decision_point = [obj.Px; obj.Py];
                        obj.Task_parameters.decision_wall = parameter(:,1); % point at which sensor detected the wall
                        obj.Task_parameters.decision_alt = obj.Task_parameters.track;
                        obj.Task_parameters.decision_handle = plot_decision(obj.Px, obj.Py);
                        obj.Task_parameters.approaching = 0;
                        obj.Task_parameters.following_up = 0;

                        obj.Task_parameters.theta = theta;
                        
                        % ---------- Register wall --------
%                         Map.Walls = [Map.Walls; [parameter(:,1)' nan nan]];
%                         wall_no = max(size(Map.Walls,1),1)
%                         obj.Map.walls_poly = [obj.Map.walls_poly, parameter(:,1)];
%                         obj.Map.discovered_walls = [obj.Map.discovered_walls, wall_no];
%                         obj.Task_parameters.decision_wall_no = wall_no;
%                         obj.Task_parameters.wall_no = wall_no;
                        
%                         [wall_cell, wall_sect] = grid_position(parameter(1,1), parameter(2,1), Map);
%                         Map = map_cell(wall_sect, wall_cell, Map, wall_no);
%                         plot_cell(wall_sect, wall_cell, Map, wall_no);
                        
%                         obj(o).Task_parameters.tracked_cells = [obj(o).Task_parameters.tracked_cells; wall_cell];
%                      obj(o).Task_parameters.tracked_sects = [obj(o).Task_parameters.tracked_sects; wall_sect];
                        % ---------------------------------
                
                    case 'new wall'
                        
                        if ( strcmp(obj.Task_parameters.track,'right') && obj.Sensor_status(3) )
                            
                            [obj.Target_xy, obj.Task_parameters.track] = ...
                                    set_new_target('left', obj.Px, obj.Py, Map, ...
                                    parameter(:,[1,3]), theta);
                                
                            % ---------- Register wall --------(1/2)
% % % %                             obj.Map.walls_poly = [obj.Map.walls_poly, parameter(:,4), parameter(:,1)];
% % % %                             Map.Walls(obj.Map.discovered_walls(end),[3,4]) = parameter(:,4)';
% % % %                             Map.Walls = [Map.Walls; [parameter(:,1)' nan nan]];
                            
                            % ---------------------------------
                            
                        else % if ( strcmp(obj(o).Task_parameters.track,'left') && obj(o).Sensor_status(2) ) 
                            
                            [obj.Target_xy, obj.Task_parameters.track] = ...
                                    set_new_target('right', obj.Px, obj.Py, Map, ...
                                    parameter(:,[1,2]), theta);
                                
                            % ---------- Register wall --------(1/2)
% % % %                             obj.Map.walls_poly = [obj.Map.walls_poly, parameter(:,5), parameter(:,1)];
% % % %                             Map.Walls(obj.Map.discovered_walls(end),[3,4]) = parameter(:,5)';
% % % %                             Map.Walls = [Map.Walls; [parameter(:,1)' nan nan]];
                            % ---------------------------------
                            
                        end
                        
                        % ---------- Register wall --------(2/2)
% % % %                         wall_no = size(Map.Walls,1);
% % % %                         obj.Map.discovered_walls = [obj.Map.discovered_walls, wall_no];
% % % %                         obj.Task_parameters.wall_no = wall_no;
% % % %                             
% % % %                         [wall_cell, wall_sect] = grid_position(parameter(1,1), parameter(2,1), Map);
% % % %                         Map = map_cell(wall_sect, wall_cell, Map, wall_no);
% % % %                         plot_cell(wall_sect, wall_cell, Map, wall_no);
                        % ---------------------------------
% %                         obj.Task_parameters.discover_wall = 0;
                        
%                         obj.Target_xy = obj.Task_xy;

                    case 'follow up'
                
                        theta = atan2d(obj.Vy,obj.Vx);
                        
                        [obj.Target_xy, obj.Task_parameters.track] = ...
                                set_new_target(obj.Task_parameters.decision_alt, obj.Px, obj.Py, Map,...
                                parameter, theta);

                        % ---------- Register wall --------                      
% % % % %                         obj.Map.walls_poly = [obj.Map.walls_poly, obj.Task_parameters.decision_wall];
% % %                         obj.Task_parameters.wall_no = obj.Task_parameters.decision_wall_no;
                        % ---------------------------------
%                           
%                         obj.Target_xy = obj.Task_xy;
    
                        subplot(3,2,[2,4]), delete(obj.Task_parameters.decision_handle),
                        obj.Task_parameters = rmfield(obj.Task_parameters,...
                            {'decision_point','decision_alt','decision_handle','decision_wall'});
                        obj.Task_parameters.following_up = 0;
                         
                end
                
        end
        
        obj.Task_xy = obj.Target_xy(:,end);
%         target = obj.Target_xy(:,end), pause
    end
    
    plot_target(obj.Target_xy(:,1), old_target, Map);
        
        if ~isempty(old_task)
            if strcmp(task, old_task), str_type = 'continue'; 
            else
                if Queue == -1
                    str_type = 'to_parent';
                else str_type = 'switch'; 
                end
            end

            string = construct_str(str_type,  o, obj, old_task, cause);
            annotate_update([],string,'set_task');
            disp('---------------------------------------------------------'),
            for line = 1:size(string,2), disp(string{line}), end, disp(' ');
%             annotate_mode(Modes_abr{strcmp(Modes,old_task)}, Modes_abr{strcmp(Modes,task)}, obj(o).Px, obj(o).Py, Map)

        end
        disp('714')
    pause(1)
    end

    function [obj,object_distances,points] = sensor_RI(obj, Sensor_walls, other_RIs_Px, other_RIs_Py, other_RIs_XVertices, other_RIs_YVertices) %Walls_xbox,Walls_ybox

        sensor_x = obj.Px*ones(obj.sensing_used,1); 
        sensor_y = obj.Py*ones(obj.sensing_used,1);

        theta = atan2d([obj.Vy],[obj.Vx]);

        sensor_x(:,2) = sensor_x(:,1) + obj.sensing_distance(1:obj.sensing_used)'.*cosd(theta - obj.sensing_angles(1:obj.sensing_used)');
        sensor_y(:,2) = sensor_y(:,1) + obj.sensing_distance(1:obj.sensing_used)'.*sind(theta - obj.sensing_angles(1:obj.sensing_used)');

        obj.Sensor_handles = plot_sensor(sensor_x, sensor_y, obj.Sensor_handles);

        points = nan(2,obj.sensing_used);
        object_distances = NaN(1,obj.sensing_used);
        obj.Sensor_status = zeros(1, obj.sensing_used);

        for sensor = 1:obj.sensing_used
            Xi = []; Yi = [];
            
            [Xi, Yi] = polyxpoly(sensor_x(sensor,:), sensor_y(sensor,:), Sensor_walls.Walls_poly(1,:), Sensor_walls.Walls_poly(2,:), 'unique'); 
            
%             for wall = 1:size(Walls_xbox,1)
%                 xbox = Walls_xbox(wall,:); ybox = Walls_ybox(wall,:);
%                 [xi, yi] = polyxpoly(sensor_x(sensor,:), sensor_y(sensor,:), xbox, ybox, 'unique'); 
%                 Xi = [Xi, xi']; Yi = [Yi, yi'];
%             end

            if ~isempty(Xi)
%                         
                obj.Sensor_status(1,sensor) = 1;

                if length(Xi)>1 %multiple intersections per sensor
                    x = [];
                    for i = 1:length(Xi), x = [x, sqrt((Xi(i)-obj.Px)^2 + (Yi(i)-obj.Py)^2)]; end
                    [object_distances(1,sensor), ind] = min(x);
                    points(:,sensor) = [Xi(ind); Yi(ind)];
                else
                    object_distances(1,sensor) = sqrt((Xi-obj.Px)^2 + (Yi-obj.Py)^2);
                    points(:,sensor) = [Xi; Yi];
                end    
            else
                points(1,sensor) = sensor_x(sensor,2);
                points(2,sensor) = sensor_y(sensor,2);
            end
        end
        
        if ~isempty(other_RIs_XVertices)
        
        other_RIs_XVertices = [other_RIs_XVertices; other_RIs_XVertices(1,:)];
        other_RIs_YVertices = [other_RIs_YVertices; other_RIs_YVertices(1,:)];
                
        for other_RI = 1:length(other_RIs_Px)
            
            if sqrt((obj.Px - other_RIs_Px(other_RI))^2 + (obj.Py - other_RIs_Py(other_RI))^2) ...
                    < (max(obj.sensing_distance(1:obj.sensing_used)) + sqrt((obj.RI_length/2)^2 + (obj.RI_width/2)^2))
                    
%                 Px = obj.Px, Py = obj.Py
%                 otherPx = other_RIs_Px(other_RI), otherPy = other_RIs_Py(other_RI)
%                 
%                 sqrt((obj.Px - other_RIs_Px(other_RI))^2 + (obj.Py - other_RIs_Py(other_RI))^2) ...
%                     < (max(obj.sensing_distance(1:obj.sensing_used)) + sqrt((obj.RI_length/2)^2 + (obj.RI_width/2)^2))
                
%             if rectint([obj.Px, obj.Py, ],[other_RIs_Px(other_RI), other_RIs_Py(other_RI), ]) > 0 
                
                for sensor = 1:obj.sensing_used
%                     other_RIs_XVertices
                    
%                     pause
                    [Xi, Yi] = polyxpoly(sensor_x(sensor,:), sensor_y(sensor,:), ...
                        other_RIs_XVertices(:,other_RI)', ...
                        other_RIs_YVertices(:,other_RI)', 'unique');
                              
                    if ~isempty(Xi)
%                         
                        obj.Sensor_status(1,sensor) = 2;

                        if length(Xi)>1 %multiple intersections per sensor
                            x = [];
                            for i = 1:length(Xi), x = [x, sqrt((Xi(i)-obj.Px)^2 + (Yi(i)-obj.Py)^2)]; end
                                [object_distances(1,sensor), ind] = min(x);
                                points(:,sensor) = [Xi(ind); Yi(ind)];
                        else
                                object_distances(1,sensor) = sqrt((Xi-obj.Px)^2 + (Yi-obj.Py)^2);
                                points(:,sensor) = [Xi; Yi];
                        end    
                    end
                end    
            end
        end
        end
    end

    function obj = update_vertices(obj)

        x = [obj.Px];
        y = [obj.Py];
        for o = 1:length(obj)
            obj(o).PX = mean(x);
            obj(o).PY = mean(y);
        end

        l = [obj.RI_length];
        w = [obj.RI_width];
        theta = atan2d([obj.Vy],[obj.Vx]);

        XX = num2cell([x-l.*cosd(theta)./2+w.*sind(theta)./2; ...
            x-l.*cosd(theta)./2-w.*sind(theta)./2; ...
            x+l.*cosd(theta)./2-w.*sind(theta)./2; ...
            x+l.*cosd(theta)./2+w.*sind(theta)./2],1);
        [obj.XVertices] = XX{:};

        YY = num2cell([y-l.*sind(theta)./2-w.*cosd(theta)./2; ...
            y-l.*sind(theta)./2+w.*cosd(theta)./2; ...
            y+l.*sind(theta)./2+w.*cosd(theta)./2; ...
            y+l.*sind(theta)./2-w.*cosd(theta)./2],1);
        [obj.YVertices] = YY{:};

    end
    
    function [obj, Map] = map_wall(obj, o, wall_xy, Map, end_str)
                        
%         [wall_cell, wall_sect] = grid_position(wall_xy(1), wall_xy(2), Map);
%         Map = map_cell(wall_sect, wall_cell, Map, 0.75 + 1i*o+1000);
%         plot_cell(wall_sect, wall_cell, Map, 0.75);

%         obj.Task_parameters.tracked_cells = wall_cell;
%         obj.Task_parameters.tracked_sects = wall_sect;

        switch end_str
            case 'midpoint'
                [wall_cell, wall_sect] = grid_position(wall_xy(1), wall_xy(2), Map);
                Map = map_cell(wall_sect, wall_cell, Map, 0.75 + 1i*o+1000);
                plot_cell(wall_sect, wall_cell, Map, 0.75);
                
                obj.Task_parameters.tracked_cells = wall_cell;
                obj.Task_parameters.tracked_sects = wall_sect;
                obj.Task_parameters.tracked_wall = [];
                
            case 'start of midpoint' % before follow up
                [wall_cell, wall_sect] = grid_position(wall_xy(1), wall_xy(2), Map);
                Map = map_cell(wall_sect, wall_cell, Map, 0.75 + 1i*o+1000);
                plot_cell(wall_sect, wall_cell, Map, 0.75);
                
%                 obj.Task_parameters.tracked_cells = wall_cell;
%                 obj.Task_parameters.tracked_sects = wall_sect;
                obj.Task_parameters.tracked_cells = [obj.Task_parameters.tracked_cells; wall_cell];
                obj.Task_parameters.tracked_sects = [obj.Task_parameters.tracked_sects; wall_sect];
                
                obj.Task_parameters.tracked_wall = [wall_xy(1), wall_xy(2)];
                
            case 'start of new wall'
%                 obj.Task_parameters.tracked_wall = wall_xy;
%                 wall_no = size(Map.Walls,1);
%                 obj.Map.discovered_walls = [obj.Map.discovered_walls, wall_no];
%                 obj.Task_parameters.wall_no = wall_no;
%                 end_str, wall_xy
%                 pause
                
                [wall_cell, wall_sect] = grid_position(wall_xy(1), wall_xy(2), Map);
                Map = map_cell(wall_sect, wall_cell, Map, 0.75 + 1i*o+1000);
                plot_cell(wall_sect, wall_cell, Map, 0.75);
%                 end_str
                obj.Task_parameters.tracked_cells = wall_cell;
                obj.Task_parameters.tracked_sects = wall_sect;
                obj.Task_parameters.tracked_wall = [wall_xy(1), wall_xy(2)];
                
                [wall_cell, wall_sect] = grid_position(wall_xy(3), wall_xy(4), Map);
                Map = map_cell(wall_sect, wall_cell, Map, 0.75 + 1i*o+1000);
                plot_cell(wall_sect, wall_cell, Map, 0.75);
                
                obj.Task_parameters.tracked_cells = [obj.Task_parameters.tracked_cells; wall_cell];
                obj.Task_parameters.tracked_sects = [obj.Task_parameters.tracked_sects; wall_sect];
                
                
%                 pause
                
            case 'end'
%                 obj.Task_parameters.tracked_wall = ...
%                     [obj.Task_parameters.tracked_wall, wall_xy];
%                 end_str
                wall_no = size(Map.Walls,1)+1; %max(size(Map.Walls,1),1)

                Map.Walls = [Map.Walls; ...
                    [obj.Task_parameters.tracked_wall, wall_xy(1), wall_xy(2)]];
%                 map_wall = Map.Walls;
                
                obj.Map.walls_poly = [obj.Map.walls_poly, obj.Task_parameters.tracked_wall', [wall_xy(1); wall_xy(2)], [nan; nan]];
                
                obj.Map.discovered_walls = [obj.Map.discovered_walls, wall_no];
                
%                 tracked_cells = obj.Task_parameters.tracked_cells
                
                for cell = 1: size(obj.Task_parameters.tracked_cells,1)
                    Map = map_cell(obj.Task_parameters.tracked_sects(cell,:), obj.Task_parameters.tracked_cells(cell,:), Map, wall_no);
                    plot_cell(obj.Task_parameters.tracked_sects(cell,:), obj.Task_parameters.tracked_cells(cell,:), Map, wall_no)
                end
%                 pause
        end

    end
    
    function display_parent_tasks(obj)
        if isfield(obj.Task_parameters,'Parent_task')
            disp(['Parent_task = ',num2str(obj.Task_parameters.Parent_task)])
            if isfield(obj.Task_parameters.Parent_task_parameters,'Parent_task')
                disp(['2nd Parent_task = ',num2str(obj.Task_parameters.Parent_task_parameters.Parent_task)])
                if isfield(obj.Task_parameters.Parent_task_parameters.Parent_task_parameters,'Parent_task')
                    disp(['3rd Parent_task = ',num2str(obj.Task_parameters.Parent_task_parameters.Parent_task_parameters.Parent_task)])
                end
            end
        end
    end
end
    
end

function [cell_xy, sect_yxstring] = grid_position(Px, Py, Map)

    Py = round(Py*1000)/1000;
    Px = round(Px*1000)/1000;

    s1=floor(Px/Map.sect_size)+1*(Px>=0);    cellx=floor((Px-(s1 - 1*(Px>=0) )*Map.sect_size)/Map.cell_size)+1;
    s2=floor(Py/Map.sect_size)+1*(Py>=0);    celly=floor((Py-(s2 - 1*(Py>=0) )*Map.sect_size)/Map.cell_size)+1;

    %             curr_cell = sprintf('%.3d%.3d',c2,c1);
    signs = {'n','p'};
    cell_xy = [cellx, celly];
    sect_yxstring = sprintf('%s%.3d%s%.3d',...
        signs{(sign(s2)>0)+1},abs(s2),signs{(sign(s1)>0)+1},abs(s1));
end

function [Px, Py] = position_on_grid(cell_xy, sect_yxstring, Map)
    Px = (cell_xy(1,1) - (Map.cell_size/2))*Map.cell_size + (strcmp(sect_yxstring(5),'p') - strcmp(sect_yxstring(5),'n'))*(str2num(sect_yxstring(6:8))-1)*Map.sect_size;
    Py = (cell_xy(1,2) - (Map.cell_size/2))*Map.cell_size + (strcmp(sect_yxstring(1),'p') - strcmp(sect_yxstring(1),'n'))*(str2num(sect_yxstring(2:4))-1)*Map.sect_size;
end
    
function Map = RI_count(old_sector, current_sector, Map)

    % Update current and old sector's RI count:
    
    Map = register_sector(current_sector, Map);
    
    eval(['Map.sect_',current_sector,'.RIs = Map.sect_',current_sector,'.RIs + 1;'])
    
    if ~isnan(old_sector)
        eval(['Map.sect_',old_sector,'.RIs = Map.sect_',old_sector,'.RIs - 1;'])
    end

end

function Map = register_sector(sector, Map)
    sector_registered = isfield(Map,['sect_',sector]);
    if ~sector_registered
        eval(['Map.sect_',sector,'.map = NaN(Map.sect_size/Map.cell_size);'])
        eval(['Map.sect_',sector,'.explored_cells = NaN(Map.sect_size/Map.cell_size);'])
        eval(['Map.sect_',sector,'.explored = 0;'])
        
        eval(['Map.sect_',sector,'.RIs = 0;'])
    end
end

function [Sensor_update, sector_overloaded] = update_freq(current_sector, Map)
            
    %Check if sector is registered:
    sector_registered = isfield(Map,['sect_',current_sector]);
    if sector_registered
        
        sector_explored = eval(['Map.sect_',current_sector,'.explored']);
        sector_busy = eval(['Map.sect_', current_sector,'.RIs > 1']);
        sector_overloaded = eval(['Map.sect_', current_sector,'.RIs > Map.sect_max']);
% % %         eval(['Map.sect_', current_sector,'.RIs']);
            
        if sector_explored
            Sensor_update = 'none';
        else
            if sector_busy
                Sensor_update = 'frequent';
            else
                Sensor_update = 'frequent';
% % % % %                 Sensor_update = 'non-frequent';
            end
        end        
        
    else % if sector is unregistered
        plot_sector(current_sector, Map.sect_size, 0);
% % % % %         Sensor_update = 'non-frequent';
        Sensor_update = 'frequent';
    end    
end

function cell_status = read_map(sector, cell, Map)

    if isfield(Map,['sect_',sector])
        cell_x = cell(2);
        cell_y = cell(1);
        cell_status = eval(['Map.sect_',sector,'.map(cell_x, cell_y)']);
    else
        cell_status = NaN;
    end
    
end

function [discovered, being_discovered, cell_status] = is_wall_discovered(obs_points, Map)

%     cell_status = zeros(1,size(obs_points,2));

%     for x = 1:size(obs_points,2)
        [cellxy, sec_yxstr] = grid_position(obs_points(1), obs_points(2), Map);
        cell_status = read_map(sec_yxstr, cellxy, Map);
%     end
    discovered = cell_status > 0.75;
    being_discovered = (real(cell_status) == 0.75);
%     cell_status
%     obs_points
%     eval(['Map.sect_',sec_yxstr,'.map'])
%     pause

end

function [sector_overloaded] = is_sector_overloaded(current_sector,Map)

    sector_registered = isfield(Map,['sect_',current_sector]);
    if sector_registered
        
%         sector_explored = eval(['Map.sect_',current_sector,'.explored']);
%         sector_busy = eval(['Map.sect_', current_sector,'.RIs > 1']);
        sector_overloaded = eval(['Map.sect_', current_sector,'.RIs > Map.sect_max']);
    else
        sector_overloaded = 0;
    end
end
        
function Map = map_cell(current_sector, current_cell, Map, cell_status)

    %Check if cell is explored:
% %     if strcmp(current_sector, 'p005p003') && cell_status >= 0.5
% %         sect_map = eval(['Map.sect_',current_sector,'.map'])
% %         pause
% %     end
%     disp('map _cell')
    if isfield(Map,['sect_',current_sector])
        
        cell_explored = ~isnan(eval(['Map.sect_',current_sector,'.explored_cells(current_cell(2),current_cell(1))']));
        read_cell_status = eval(['Map.sect_',current_sector,'.map(current_cell(2),current_cell(1))']);

        if ~cell_explored || (cell_status > read_cell_status) || (read_cell_status > 1000)%(cell_status > 0.5) || (cell_status == 0.5 && read_cell_status == 0)
            
            eval(['Map.sect_',current_sector,'.explored_cells(current_cell(2),current_cell(1)) = 1;'])
            eval(['Map.sect_',current_sector,'.map(current_cell(2),current_cell(1)) = cell_status;'])

            if ~cell_explored || ((cell_status > read_cell_status) && (cell_status < 1000))
                if cell_status > 1000
                   disp('Ya nhar Abyad ?!!') %tailored
                   cell_status = 0;
%                    pause
                end
                plot_cell(current_sector, current_cell, Map, cell_status);
            end
            % If all cells are explored, then sector is explored
            if all(all(eval(['Map.sect_',current_sector,'.explored_cells'])==1))
                eval(['Map.sect_',current_sector,'.explored = 1;'])
                plot_sector(current_sector, Map.sect_size, 1);
            end 
        end
        
    end
    
end

function Map = update_map(current_sector, Map, sensed, not_sensed)

    if ~isempty(sensed)
        Xi = sensed(1,:); Yi = sensed(2,:);
        for x = 1:length(Xi)
%             Xi(x), Yi(x),
            [obs_cellxy, obs_sec_yxstr] = grid_position(Xi(x), Yi(x), Map);

            if strcmp(obs_sec_yxstr,current_sector)

                Map = map_cell(current_sector, obs_cellxy, Map, 0.5);

            else
                Sensor_update = update_freq(obs_sec_yxstr, Map);
                if ~strcmp(Sensor_update,'none')
                    Map = register_sector(obs_sec_yxstr, Map);
                    Map = map_cell(obs_sec_yxstr, obs_cellxy, Map, 0.5);
                end
            end
        end
    end
    if ~isempty(not_sensed)
        Xn = not_sensed(1,:); Yn = not_sensed(2,:);
        for x = 1:length(Xn)
%             Xn(x), Yn(x)
            [cellxy, sec_yxstr] = grid_position(Xn(x), Yn(x), Map);

            if strcmp(sec_yxstr,current_sector)

                Map = map_cell(current_sector, cellxy, Map, 0);

            else
                Sensor_update = update_freq(sec_yxstr, Map);
                if ~strcmp(Sensor_update,'none')
                    Map = register_sector(sec_yxstr, Map);
                    Map = map_cell(sec_yxstr, cellxy, Map, 0);
                end
            end
        end
    end
% % % %     pause
end

function plot_sector(sector,Sector_size,Sector_type)

    subplot(3,2,[2,4]), hold on,
%             sector = Map.reg_sectors(x,:);
    sector_y = (-1*(strcmp(sector(1),'n'))*str2double(sector(2:4)) + ...
        (strcmp(sector(1),'p'))*(str2double(sector(2:4))-1))*Sector_size;
    sector_x = (-1*(strcmp(sector(5),'n'))*str2double(sector(6:8)) + ...
        (strcmp(sector(5),'p'))*(str2double(sector(6:8))-1))*Sector_size;
    
    if Sector_type == 0, color = 'y'; else color = 'g'; end
    
    rectangle('Position',[sector_x,sector_y,Sector_size,Sector_size],'EdgeColor',color,'FaceColor','none')

end

function plot_cell(sector, cell_xy, Map, cell_status)

    subplot(3,2,[2,4]), hold on,
%             sector = Map.reg_sectors(x,:);
    sector_y = (-1*(strcmp(sector(1),'n'))*str2double(sector(2:4)) + ...
        (strcmp(sector(1),'p'))*(str2double(sector(2:4))-1))*Map.sect_size;
    sector_x = (-1*(strcmp(sector(5),'n'))*str2double(sector(6:8)) + ...
        (strcmp(sector(5),'p'))*(str2double(sector(6:8))-1))*Map.sect_size;

    cell_x = sector_x + (cell_xy(1)-1)*Map.cell_size;
    cell_y = sector_y + (cell_xy(2)-1)*Map.cell_size;
    
    if cell_status == 0, color = 'w'; % else color = 'k'; end
    else 
        if cell_status == 1, color = 'r';
        elseif cell_status == 2, color = 'b';
        elseif cell_status == 3, color = 'g';
        elseif cell_status == 4, color = 'm';
        elseif cell_status == 5, color = 'k';
%         elseif cell_status > 100, color = [0.5 0.5 0.5];
        elseif cell_status == 0.5, color = [0.5, 0.5, 0.5];
        elseif real(cell_status) == 0.75, color = 'y';
        else
            color=[0.25 0.25 0.25];
            disp(cell_status)
            pause
        end
    end
    
    rectangle('Position',[cell_x,cell_y,Map.cell_size,Map.cell_size],'EdgeColor','k','FaceColor',color)

end

% function handle = plot_RI(handle, XVertices, YVertices, color)
function plot_RI(XVertices, YVertices, color, Px, Py, Pz)

    persistent handle1 handle2 handle3 handle4
    %--------------
    subplot(3,2,[1,3]),
    %--------------
    delete(handle1);        
    handle1 = fill([XVertices], [YVertices],'b'); % plot RIs
    
    %--------------
    subplot(3,2,[2,4]), hold on,
    %--------------
    delete(handle2)
%     XVertices
%     color
    handle2 = fill([XVertices], [YVertices], 'b');%, [color]); % plot RIs
    
    %--------------
    subplot(3,2,5),
    %--------------
    delete(handle3); delete(handle4);
    
    handle3 = fill3([XVertices], [YVertices], [Pz;Pz;Pz;Pz],'b'); % plot RIs
    handle4 = stem3([Px], [Py], [Pz], ':b','Marker','none');
    pause(0.00001)

end

function plot_target(target_xy, old_target_xy, Map)

    if ~isnan(target_xy)
        [target_cell_xy, target_sector] = grid_position(target_xy(1), target_xy(2), Map);

        subplot(3,2,[2,4]), hold on,
    %             sector = Map.reg_sectors(x,:);
        target_sector_y = (-1*(strcmp(target_sector(1),'n'))*str2double(target_sector(2:4)) + ...
            (strcmp(target_sector(1),'p'))*(str2double(target_sector(2:4))-1))*Map.sect_size;
        target_sector_x = (-1*(strcmp(target_sector(5),'n'))*str2double(target_sector(6:8)) + ...
            (strcmp(target_sector(5),'p'))*(str2double(target_sector(6:8))-1))*Map.sect_size;

        target_cell_x = target_sector_x + (target_cell_xy(1)-1)*Map.cell_size;
        target_cell_y = target_sector_y + (target_cell_xy(2)-1)*Map.cell_size;
        
        rectangle('Position',[target_cell_x,target_cell_y,Map.cell_size,Map.cell_size],'EdgeColor','r','FaceColor','none')

        if ~isempty(old_target_xy)

            [old_target_cell_xy, old_target_sector] = grid_position(old_target_xy(1), old_target_xy(2), Map);

            old_target_sector_y = (-1*(strcmp(old_target_sector(1),'n'))*str2double(old_target_sector(2:4)) + ...
                (strcmp(old_target_sector(1),'p'))*(str2double(old_target_sector(2:4))-1))*Map.sect_size;
            old_target_sector_x = (-1*(strcmp(old_target_sector(5),'n'))*str2double(old_target_sector(6:8)) + ...
                (strcmp(old_target_sector(5),'p'))*(str2double(old_target_sector(6:8))-1))*Map.sect_size;

            old_target_cell_x = old_target_sector_x + (old_target_cell_xy(1)-1)*Map.cell_size;
            old_target_cell_y = old_target_sector_y + (old_target_cell_xy(2)-1)*Map.cell_size;

            rectangle('Position',[old_target_cell_x,old_target_cell_y,Map.cell_size,Map.cell_size],'EdgeColor','k','FaceColor','none')
        end
    end
end

function handle = plot_sensor(sensor_x,sensor_y,handle)

    subplot(3,2,[2,4]), hold on,
%             sector = Map.reg_sectors(x,:);
    delete(handle),
    
    handle = plot(sensor_x(1,:),sensor_y(1,:),...
        sensor_x(2,:),sensor_y(2,:),...
        sensor_x(3,:),sensor_y(3,:),...
        sensor_x(4,:),sensor_y(4,:),...
        sensor_x(5,:),sensor_y(5,:));

end

function handle = plot_decision(x,y)
    
    subplot(3,2,[2,4]), hold on,
    handle = plot(x,y,'*r');
    
end

function str = construct_str(MSG_type,o,obj,old_task, cause)
    Modes           = {'Explore', 'Traverse', 'Track', 'Anchor'};
    Modes_abr       = {'EP0'    , 'EP1'     , 'DPx'  , 'Anc'   };

    switch MSG_type
        case 'display'
            str = {...
                ['RI ',num2str(o),' (', Modes_abr{strcmp(Modes,obj.Task)},')'],...
                ['x = ',num2str(round(obj.Px,3,'significant')),', y = ',num2str(round(obj.Py,3,'significant'))], ...
                ['x = ',num2str(round(obj.Target_xy(1),2,'significant')),', y = ', num2str(round(obj.Target_xy(2),2,'significant'))]...
                ['x = ',num2str(round(obj.Vx,2,'significant')),', y = ',num2str(round(obj.Vy,3,'significant'))], ...
                [num2str(round(atan2d(obj.Vy,obj.Vx),2,'significant'))],...
                [mat2str(obj.Sensor_status)]};
%             str = {['RI no.',num2str(o),':'], ...
%                 ['Mode    : "', obj.Task, '"']...
%                 ['Position: x = ',num2str(obj.Px),', y = ',num2str(obj.Py)], ...
%                 ['Target  : x = ',num2str(obj.Target_xy(1)),', y = ', num2str(obj.Target_xy(2))]...
%                 ['Velocity: x = ',num2str(obj.Vx),', y = ',num2str(obj.Vy),', theta = ',num2str(atan2d(obj.Vy,obj.Vx))]...
%                 ['Sensors : ', mat2str(obj.Sensor_status)]};
        case 'switch'
            str = {['RI no.',num2str(o),' is switching from "', old_task,'" to "', obj.Task, '" because ', cause]};%,...
%                 ['Position: x = ',num2str(obj.Px),', y = ',num2str(obj.Py)],...
%                 ['Target  :   x = ',num2str(obj.Task_xy(1)),', y = ', num2str(obj.Task_xy(2)),'...'],...
%                 };
%             str = {str, [cause]};
        case 'continue'
            str = {['RI no.',num2str(o),' is continuing in "', obj.Task,'" task because ', cause, '...']};%,...
%                 ['Position: x = ',num2str(obj.Px),', y = ',num2str(obj.Py)],...
%                 ['Target  :   x = ',num2str(obj.Task_xy(1)),', y = ', num2str(obj.Task_xy(2)),'...'],...
%                 };
%             str = {str, [cause]};
        case 'to_parent'
            str = {['RI no.',num2str(o),' is switching from "', old_task,'" back to parent task "', obj.Task, '" because ',  cause]};%,...
%                 ['Position: x = ',num2str(obj.Px),', y = ',num2str(obj.Py)],...
%                 ['Target  :   x = ',num2str(obj.Task_xy(1)),', y = ', num2str(obj.Task_xy(2)),'...'],...
%                 };
%             str = {str, [cause]};
    end
    
end

function annotate_mode(old_mode, new_mode, Px, Py, Map)

    px = (Px-0.5)/Map.W;
%     if px >= 0.5, start_x = .3; else start_x = .8; end
    py = (Py-0.5)/Map.L;
%     if py >= 0.5, start_y = .3; else start_y = .8; end
    
    subplot(3,2,[2,4]), hold on,
    subplot_info = get(gca,'Position');
    plot_start_x = subplot_info(1); plot_dx = subplot_info(3); 
    plot_start_y = subplot_info(2); plot_dy = subplot_info(4);
    
    handle = annotation('textarrow','Position',[plot_start_x, plot_start_y, px*plot_dx, py*plot_dy],... %[0.3,0.5],[0.6,0.5],...
        'String',[old_mode,' to ',new_mode,' ']);
        
    delete(handle)
end

function handle = annotate_update(handle, string, type, o)
    persistent header_handle set_task_handle
    subplot(3,2,6), axis off,
    subplot_info = get(gca,'Position');
    
    switch type
        case 'display'
            header = {...
                ['        '],
                ['Px, Py: '],
                ['Tx, Ty: '],
                ['Vx, Vy: '],
                ['Theta : '],
                ['Sensor: ']};

            delete(handle), delete(header_handle)
%             subplot_info = get(gca,'Position');
            header_plot_info = [subplot_info(1), subplot_info(2), subplot_info(3)/6, 6*subplot_info(4)/8];
            str_plot_info = [subplot_info(1)+(2.5*(o-1)+1)*subplot_info(3)/6, subplot_info(2), 2.5*subplot_info(3)/6, 6*subplot_info(4)/8];

            header_handle = annotation('textbox','Position',header_plot_info,'VerticalAlignment','middle','FontWeight','bold','String',header);
            handle = annotation('textbox', 'Position', str_plot_info,'HorizontalAlignment','center',...
               'VerticalAlignment','middle','Color','b','FontSize',9,'String', string);
        case 'set_task'
            delete(set_task_handle)
            set_task_plot_info = [subplot_info(1), subplot_info(2)+6*subplot_info(4)/8, subplot_info(3), 2*subplot_info(4)/8];
            set_task_handle = annotation('textbox', 'Position', set_task_plot_info,'HorizontalAlignment','center',...
               'VerticalAlignment','middle','Color','r','String', string);
    end
end

function [target_xy, track] = set_new_target(Type,x,y,Map,points,theta)

    switch Type
        case {'right or left','right','left'}
            selections = {'right', 'left'};
            if strcmp(Type,'right or left')
                s = randi(2,1,1);
                selection = selections{s};
            else
                selection = Type;
            end
            
            track = selections{~strcmp(selections,selection)};
%                     wall_slope = (Yi(2) - Yi(1))/(Xi(2) - Xi(1));
%                     points(2,2), points(2,1),
%                     points(1,2), points(1,1),
                    
%                     theta
                    theta_wall = atan2d(...
                        (round((points(2,2) - points(2,1))*1000)/1000),( round(1000*(points(1,2) - points(1,1)))/1000));
            switch selection
                case 'right'
                    target_xy = [x + sign(sind(theta))*Map.RSSI_range*abs(cosd(theta_wall)); ...
                        y - sign(cosd(theta))*Map.RSSI_range*abs(sind(theta_wall))];
%                     target_xy = [x + Map.cell_size*cosd(theta_wall), y + Map.cell_size*sind(theta_wall)];
                case 'left'
                    target_xy = [x - sign(sind(theta))*Map.RSSI_range*abs(cosd(theta_wall));...
                        y + sign(cosd(theta))*Map.RSSI_range*abs(sind(theta_wall))];
%                     target_xy = [x - Map.cell_size*cosd(theta_wall), y - Map.cell_size*sind(theta_wall)];
            end
            
%             if theta == -90 || theta == 90,
%                 theta
%                 theta_wall
%                 target_xy
%                 pause
%             end    
            
            disp([' - Going ', selection, ' and tracking the ', track, ' sensor...'])
            disp(' - New target: ')
            disp(['   x = ', num2str(target_xy(1)), '    y = ', num2str(target_xy(2))])
            pause(1)
            
        case 'random'
            track = [];
            
            [~, sector] = grid_position(x, y, Map);
            is_current_sector_explored = eval(['Map.sect_',sector,'.explored']);
            if is_current_sector_explored
            
                xi = 1;
                target_x = inf; target_y = inf;
                is_target_sector_explored = 1;

%                 walls = points.walls;
                walls_poly = points.walls_poly;

    % % %             w = (x^2 + y^2 > Map.RSSI_range^2)
    % % %             t = (x < Map.min_hor)
    % % %             f = (y < Map.min_ver)
                while (target_x^2 + target_y^2 > Map.RSSI_range^2) ...
                        || (target_x < Map.min_ver) || (target_y < Map.min_hor) ...
                        || is_target_sector_explored ...
                        || ~isempty(xi)

                    target_x = rand(1)*Map.RSSI_range*(-1)^randi(2,1);
                    target_y = rand(1)*Map.RSSI_range*(-1)^randi(2,1);

%                     target_x^2 + target_y^2 > Map.RSSI_range^2;

                    [~, sector] = grid_position(target_x, target_y, Map);

                    if ~isempty(walls_poly)
                        [xi, ~] = polyxpoly([x,target_x], [y,target_y], ...
                            walls_poly(1,:), walls_poly(2,:), 'unique') ;
                    else
                        xi = [];
                    end

                    if isfield(Map,['sect_',sector])
                        is_target_sector_explored = eval(['Map.sect_',sector,'.explored']);
                    else
                        is_target_sector_explored = 0;
                    end
    %                 pause
                end
            else
                Sector_map = eval(['Map.sect_',sector,'.map']);
                
% % %                 Sector_map
                
                [target_cell_y, target_cell_x] = find(isnan(Sector_map));
                
                [target_x, target_y] = position_on_grid([target_cell_x, target_cell_y], sector, Map);
%                 target_x = (target_cell_x-(Map.cell_size/2))*Map.cell_size + (strcmp(sector(5),'p') - strcmp(sector(5),'n'))*(str2num(sector(6:8))-1)*Map.sect_size
%                 target_y = (target_cell_y-(Map.cell_size/2))*Map.cell_size + (strcmp(sector(1),'p') - strcmp(sector(1),'n'))*(str2num(sector(2:4))-1)*Map.sect_size                
% pause

            end
%             pause
%             target_xy = [target_x'; target_y'];
            target_xy = [target_x(randi(length(target_x),1,1)); target_y(randi(length(target_y),1,1))];
            
        case 'reflect'
            
    end
end

function target_xy = set_new_course(x,y,Map)



end

function point = fourth_point(RI_position,start_point,end_point)

    center_area = (start_point+end_point)/2;
    point = 4*center_area-start_point-end_point-RI_position;

end