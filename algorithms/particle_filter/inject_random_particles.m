function [particles] = inject_random_particles(particles, public_vars, read_only_vars, injection_rate)
    N = size(particles, 1);
    N_random = round(N * injection_rate);
    
    if N_random > 0

        random_particles = init_particle_filter(read_only_vars, public_vars, N_random);

        start_idx = N - N_random + 1;
        
        particles(start_idx:end, :) = random_particles;
    end
end