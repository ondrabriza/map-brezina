function particles = init_particles_from_kalman(public_vars, read_only_vars)
    mu = public_vars.mu;
    sigma = public_vars.sigma;
    num_particles = read_only_vars.max_particles;
    
    std_devs = sqrt(diag(sigma))';
    particles = repmat(mu, num_particles, 1) + randn(num_particles, 3) .* std_devs;
    particles(:, 3) = mod(particles(:, 3), 2*pi);

end