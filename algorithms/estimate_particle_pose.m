function pose = estimate_particle_pose(particles)
    mean_x = mean(particles(:, 1));
    mean_y = mean(particles(:, 2));

    mean_theta = mod(atan2(mean(sin(particles(:, 3))), mean(cos(particles(:, 3)))), 2*pi);
    
    pose = [mean_x, mean_y, mean_theta];
end