function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

    %init
    path = [];

    % Check if we have a valid pose (not empty, no NaNs)
    if isempty(public_vars.estimated_pose) || any(isnan(public_vars.estimated_pose))
        return; 
    end

    % map(1) == x=0, x=0
    map = read_only_vars.discrete_map.map;
    [map_size_y, map_size_x] = size(map);

    step = read_only_vars.map.discretization_step;


    % Bigger walls 
    desired_clearance = 0.6;
    kernel_size = 2 * ceil(desired_clearance/step) + 1;
    kernel = ones(kernel_size, kernel_size);
    blurred_map = conv2(map, kernel, 'same');
    map(blurred_map > 0) = 1;

    % Start position mapped to discrete mao
    start_pos = public_vars.estimated_pose; 
    start_discrete = floor(start_pos(1:2) / step) + 1;
    
    % discrete goal
    goal_discrete = read_only_vars.discrete_map.goal;

    % Safety checks for map boundaries and collisions
    if start_discrete(1) < 1 || start_discrete(1) > map_size_x || start_discrete(2) < 1 || start_discrete(2) > map_size_y
        %disp('Start position is out of map bounds!');
        return;
    end
    if map(start_discrete(2), start_discrete(1)) == 1
        %disp('Start position is inside an obstacle!');
        return;
    end
    if map(goal_discrete(2), goal_discrete(1)) == 1
        %disp('Goal position is inside an obstacle!');
        return;
    end
    
    
    g_score = inf(map_size_y, map_size_x);
    g_score(start_discrete(2), start_discrete(1)) = 0;
    
    f_score = inf(map_size_y, map_size_x);
    f_score(start_discrete(2), start_discrete(1)) = norm(start_discrete - goal_discrete);
    
    came_from_x = zeros(map_size_y, map_size_x);
    came_from_y = zeros(map_size_y, map_size_x);
    
    open_set = start_discrete; 
    
    in_open_set = false(map_size_y, map_size_x);
    in_open_set(start_discrete(2), start_discrete(1)) = true;
    closed_set = false(map_size_y, map_size_x);
    
    % Directions and costs
    dx = [1, 1, 0, -1, -1, -1, 0, 1];
    dy = [0, 1, 1, 1, 1, 0, -1, -1];
    costs = [1, sqrt(1), 1, sqrt(1), 1, sqrt(1), 1, sqrt(1) ];


    while ~isempty(open_set)
        
        % Select node with the lowest f_score
        open_indices = sub2ind([map_size_y, map_size_x], open_set(:,2), open_set(:,1)); % matrix to linear indicies
        [~, min_idx] = min(f_score(open_indices));
        current = open_set(min_idx, :); 
        
        % Check if goal is reached => RECONSTRUCT PATH
        if current(1) == goal_discrete(1) && current(2) == goal_discrete(2)
            path_discrete = current;
            curr = current;
            while ~(curr(1) == start_discrete(1) && curr(2) == start_discrete(2))
                px = came_from_x(curr(2), curr(1));
                py = came_from_y(curr(2), curr(1));
                curr = [px, py];
                path_discrete = [curr; path_discrete]; 
            end

            % Convert discrete path back to continuous
            path = (path_discrete - 0.5) * step;
            return; 
        end
        
        % Move current node from open to closed set
        open_set(min_idx, :) = [];
        in_open_set(current(2), current(1)) = false;
        closed_set(current(2), current(1)) = true;

        % Explore all 8 neighbors
        for i = 1:8
            nx = current(1) + dx(i);
            ny = current(2) + dy(i);
            
            % Check map boundaries
            if nx >= 1 && nx <= map_size_x && ny >= 1 && ny <= map_size_y
                
                % Skip obstacles and nodes in closed_set
                if map(ny, nx) == 1 || closed_set(ny, nx)
                    continue;
                end
                
                % Prevent diagonal corner-cutting
                if dx(i) ~= 0 && dy(i) ~= 0
                    wall_x = map(current(2), current(1) + dx(i)) == 1;
                    wall_y = map(current(2) + dy(i), current(1)) == 1;
                    
                    if wall_x && wall_y
                        continue; 
                    end
                end
                
                % Calculate tentative g_score
                tentative_g = g_score(current(2), current(1)) + costs(i);
                
                % If a shorter path to the neighbor is found
                if tentative_g < g_score(ny, nx)
                    came_from_x(ny, nx) = current(1);
                    came_from_y(ny, nx) = current(2);
                    g_score(ny, nx) = tentative_g;
                    
                    h = norm([nx, ny] - goal_discrete); 
                    f_score(ny, nx) = tentative_g + h;
                    
                    % Add to open set if not already present
                    if ~in_open_set(ny, nx)
                        open_set(end+1, :) = [nx, ny];
                        in_open_set(ny, nx) = true;
                    end
                end
            end
        end
    end
    
    disp('Path not found.');
end