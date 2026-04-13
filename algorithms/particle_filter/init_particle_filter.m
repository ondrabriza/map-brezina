function [particles] = init_particle_filter(read_only_vars, public_vars, N)
%INIT_PARTICLE_FILTER Summary of this function goes here

    % map_limits
    x_min = read_only_vars.map.limits(1);
    y_min = read_only_vars.map.limits(2);
    x_max = read_only_vars.map.limits(3);
    y_max = read_only_vars.map.limits(4);
    
    % random X positions within limits
    random_x = x_min + rand(N, 1) * (x_max - x_min);
    
    % random Y positions within limits
    random_y = y_min + rand(N, 1) * (y_max - y_min);
    
    % random orientations
    random_theta = rand(N, 1) * 2 * pi;
    
    % store particles [x, y, theta]
    particles = [random_x, random_y, random_theta];
end

