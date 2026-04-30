function [estimated_pose] = estimate_pose(public_vars, read_only_vars)


    estimated_pose = [];

    % Both filters are enabled: compare uncertainties, 
    if public_vars.pf_enabled && public_vars.kf_enabled

        kalman_mean = public_vars.mu;
        kalman_cov  = public_vars.sigma;

        [particle_mean, particle_sigma] = estimate_particle_pose(public_vars, read_only_vars);
        
        uncertainty_kalman = det(kalman_cov);
        uncertainty_particles = det(particle_sigma);
        
        % Evaluate and select the better estimate ---
        if uncertainty_particles < uncertainty_kalman
            estimated_pose = particle_mean;
        else
            estimated_pose = kalman_mean;
        end
        
    % Only Particle filter is enabled
    elseif public_vars.pf_enabled
        [estimated_pose, ~] = estimate_particle_pose(public_vars, read_only_vars);
        
    % Only Kalman filter is enabled
    elseif public_vars.kf_enabled
        estimated_pose = public_vars.mu;
    end
end