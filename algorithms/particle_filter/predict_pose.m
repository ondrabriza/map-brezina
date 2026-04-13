function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)

    dt = read_only_vars.sampling_period; 
    wheel_dist = read_only_vars.agent_drive.interwheel_dist;

    % scale for noise
    scale_factor = 0.1;
    
    noisy_right_velocity = motion_vector(1) + randn() * scale_factor;
    noisy_left_velocity = motion_vector(2) + randn() * scale_factor;
    

    % traveled distance by wheels
    distance_left = noisy_left_velocity * dt;
    distance_right = noisy_right_velocity * dt;

    % calc translation (delta_position) and rotation (delta_theta)
    delta_position = (distance_right + distance_left) / 2;
    delta_theta = (distance_right - distance_left) / wheel_dist;

    % update position
    new_x = old_pose(1) + delta_position * cos(old_pose(3));
    new_y = old_pose(2) + delta_position * sin(old_pose(3));
    new_theta = old_pose(3) + delta_theta;

    % map boundary
    % map.limits [x_min, y_min, x_max, y_max]
    limits = read_only_vars.map.limits;

    new_x = max(limits(1), min(new_x, limits(3)));
    new_y = max(limits(2), min(new_y, limits(4)));

    % normalize angle [0, 2*pi]
    new_theta = wrapTo2Pi(new_theta);

    new_pose = [new_x, new_y, new_theta];
end