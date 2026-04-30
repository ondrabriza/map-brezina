function [public_vars] = plan_motion(read_only_vars, public_vars)
   
    if isempty(public_vars.path) % safety check
        public_vars.motion_vector = [0, 0];
        return; 
    end
    
    %get estimated position
    robot_x = public_vars.estimated_pose(1);
    robot_y = public_vars.estimated_pose(2);
    robot_theta = public_vars.estimated_pose(3);

    % get path
    path = public_vars.path;
    
    % Pure pursuit parameter
    lookahead_distance = 0.4;
    
    % Regulator parameters
    base_wheel_speed = 0.6;
    K_p = 0.6;
    K_i = 0.05;
    K_d = 0.4;

    max_I = 2.0;

    if ~isfield(public_vars, "I_err") 
        public_vars.I_err = 0;
    end
    if ~isfield(public_vars, "prev_error")
        public_vars.prev_error = 0;
    end

    

    % speed up slowly
    ramp_samples = 50;
    if read_only_vars.counter <= ramp_samples 
        ramp_factor = read_only_vars.counter / ramp_samples;
    else
        ramp_factor = 1;
    end
    
    % Aplikujeme na rychlost
    base_wheel_speed = base_wheel_speed * ramp_factor;

    

    
    % Find all distances 
    distances = sqrt((path(:,1) - robot_x).^2 + (path(:,2) - robot_y).^2);
   
    % find minimal distance to path
    [min_dist, closest_idx] = min(distances);

    % Stop and reset particle filter and planned path if we are are in the
    % end or too far from path 
    stop_tolerance = 0.1;     
    if distances(end) < stop_tolerance || min_dist > 0.5
        public_vars.path = []; 
        public_vars.motion_vector = [0, 0]; 
        public_vars.pf_initialized = 0;
        
        public_vars.I_err = 0; % reset I
        
        return;
    end
    
    % move the waypoint in front of the robot to the minimum lookahead distance
    target_idx = closest_idx;
    for i = closest_idx:length(distances)
        if distances(i) >= lookahead_distance
            target_idx = i;
            break;
        end
    end
    
    % if the last point of the route is less then lookahead_distance, ignore lookahead distance
    dist_to_goal = sqrt((path(end, 1) - robot_x)^2 + (path(end, 2) - robot_y)^2);
    if dist_to_goal < lookahead_distance
        target_idx = size(path, 1);
    end
    
    target_x = path(target_idx, 1);
    target_y = path(target_idx, 2);
    
    % calculate angle diff 
    % atan2 returns anlge in radians in interval [-pi, pi]
    alpha_global = atan2(target_y - robot_y, target_x - robot_x);
    angle_diff = alpha_global - robot_theta;
    error = atan2(sin(angle_diff), cos(angle_diff)); 
    
    P = K_p * error;

    % anti wind-up
    if sign(error) ~= sign(public_vars.prev_error)
        public_vars.I_err = 0;
    end

    public_vars.I_err = public_vars.I_err + error;
    public_vars.I_err = max(min(public_vars.I_err, max_I), -max_I);
    
    I = K_i * public_vars.I_err;

    derivative = error - public_vars.prev_error;
    D = K_d * derivative;
    
    
    public_vars.prev_error = error; % error for next cycle
    
    turn_effort = P + I + D;

    x_min = read_only_vars.map.limits(1);
    y_min = read_only_vars.map.limits(2);
    x_max = read_only_vars.map.limits(3);
    y_max = read_only_vars.map.limits(4);
    
    % check if we are near borders
    dist_left   = robot_x - x_min;
    dist_right  = x_max - robot_x;
    dist_bottom = robot_y - y_min;
    dist_top    = y_max - robot_y;
    
    % Find minimal distance
    min_edge_dist = min([dist_left, dist_right, dist_bottom, dist_top]);
    border_safe_dist = 0.5; % "safe" distance
    
    if min_edge_dist < border_safe_dist
        %slow down
        edge_speed_factor = max(0.01, min_edge_dist / border_safe_dist);
        base_wheel_speed = base_wheel_speed * edge_speed_factor;
    end


    % Check if we are near walls
    lidar_dists = read_only_vars.lidar_distances;
    [min_lidar, min_idx] = min(lidar_dists);
    
    %Slow down if we are closer than safe_dist
    lidar_safe_dist = 0.5;
    if min_lidar < lidar_safe_dist
        
        speed_factor = max(0.01, min_lidar / lidar_safe_dist);
        base_wheel_speed = base_wheel_speed * speed_factor;
        
        % angle of minimal distance
        observation_angle = read_only_vars.lidar_config(min_idx);
        
        % [-pi, pi]
        obs_angle_wrapped = atan2(sin(observation_angle), cos(observation_angle));
        
        % Only [-90°, 90°] affects repulsion
        if abs(obs_angle_wrapped) < (pi / 2)
            
            % Closer the wall, higher repulsion
            repulsion_strength = 1.2*(1 - (min_lidar / lidar_safe_dist)); % * 0.8;
            
            turn_effort = turn_effort - sign(obs_angle_wrapped) * repulsion_strength;
        end
    end


    v_right  = base_wheel_speed + turn_effort;
    v_left = base_wheel_speed - turn_effort;

    % low-pass filter
    alfa = 0.9; 
    v_right  = alfa * v_right  + (1 - alfa) * public_vars.motion_vector(1);
    v_left = alfa * v_left + (1 - alfa) * public_vars.motion_vector(2);

    % Saturation to max_velocity
    max_velocity = read_only_vars.agent_drive.max_vel;
    max_requested = max(abs(v_right), abs(v_left));

    if max_requested > max_velocity
        scale_factor = max_velocity / max_requested;
        
        v_right  = v_right * scale_factor;
        v_left = v_left * scale_factor;
    end


    public_vars.motion_vector = [v_right, v_left];
end