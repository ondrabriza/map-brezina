function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

    N = length(lidar_config);
    measurement = zeros(1, N);
    
    ray_origin = pose(1:2);
    particle_theta = pose(3);
    walls = map.walls;
    
    for i = 1:N
        ray_direction = particle_theta + lidar_config(i);
        
        % normalize to [0, 2*pi]
        ray_direction = wrapTo2Pi(ray_direction);
        
        % calc intersections
        intersections = ray_cast(ray_origin, walls, ray_direction);
        

        if ~isempty(intersections)
            dx = intersections(:, 1) - ray_origin(1);
            dy = intersections(:, 2) - ray_origin(2);
            distances = sqrt(dx.^2 + dy.^2);

            measurement(i) = min(distances);
        else
            % if the ray hits nothing, set the distance to Inf
            measurement(i) = Inf;
        end
    end

end

