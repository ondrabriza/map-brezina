function [particles] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Summary of this function goes here

    particles = public_vars.particles;
    
    % I. Prediction
    for i=1:size(particles, 1)
        particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
    end
    
    % II. Correction
    measurements = zeros(size(particles,1), length(read_only_vars.lidar_config));
    for i=1:size(particles, 1)
        measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
    end
    weights = weight_particles(measurements, read_only_vars.lidar_distances);
    
    N = size(particles, 1);
    N_eff = 1 / sum(weights.^2);
    
    if N_eff < (N / 2)
        % III.a. Resampling
        particles = resample_particles(particles, weights);
        
        % III.b.  Inject random particlesas as part of Resampling
       
        particles = inject_random_particles(particles, public_vars, read_only_vars);
    end
end

