function [pose, sigma] = estimate_particle_pose(public_vars, read_only_vars)
    % we need to remove injected particles 
    N = read_only_vars.max_particles;
    N_random = round(N * public_vars.injection_rate);
    pose = [];
    sigma = [];
    if N_random > 0
        end_index = N - N_random;
    else 
        end_index = N;
    end

    particles = public_vars.particles(1:end_index, :);

    % mean of X,Y position
    mean_x = mean(particles(:, 1));
    mean_y = mean(particles(:, 2));
    %mean of theta in range [0, 2*pi]
    mean_theta = mod(atan2(mean(sin(particles(:, 3))), mean(cos(particles(:, 3)))), 2*pi);
    
    pose = [mean_x, mean_y, mean_theta];
    
    % Calculate sigma for later use if we need to change localization
    % algorithm from indoor to kalman
    diffs = particles - pose;
    diffs(:, 3) = mod(atan2(sin(diffs(:, 3)), cos(diffs(:, 3))), 2*pi); 
    sigma = (diffs' * diffs) / (size(particles,1) - 1);
end