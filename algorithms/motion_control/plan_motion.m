function [public_vars] = plan_motion(read_only_vars, public_vars)
   
    if ~isfield(public_vars, 'highest_idx')
        public_vars.highest_idx = 1; 
    end
    
    % % get robot position
    % robot_x = read_only_vars.mocap_pose(1);
    % robot_y = read_only_vars.mocap_pose(2);
    % robot_theta = read_only_vars.mocap_pose(3);

    robot_x = public_vars.estimated_pose(1);
    robot_y = public_vars.estimated_pose(2);
    robot_theta = public_vars.estimated_pose(3);

    % get path
    path = public_vars.path;
    
    % Pure pursuit parameter
    lookahead_distance = 0.5;
    
    % Regulator parameters
    base_wheel_speed = 0.8;
    K_p = 0.8;
    
    % Find all distances 
    distances = sqrt((path(:,1) - robot_x).^2 + (path(:,2) - robot_y).^2);
    
    % "forget" the path where the robot has already been 
    %search_start = public_vars.highest_idx;
    
    % uncomment this if you want robot to ride path from start else it
    % starts in nearest point on the track
    %search_end = min(size(path, 1), search_start + 50);
    %[~, local_idx] = min(distances(search_start:search_end));
    
    [~, closest_idx] = min(distances);
    %closest_idx = search_start + local_idx - 1;
    
    
    % if closest_idx > public_vars.highest_idx
    %     public_vars.highest_idx = closest_idx;
    % end
    
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
    
    turn_effort = K_p * error;
    
    v_left  = base_wheel_speed + turn_effort;
    v_right = base_wheel_speed - turn_effort;

    public_vars.motion_vector = [v_left, v_right];
end