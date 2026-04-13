function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here

    N = size(particles, 1);
    new_particles = zeros(size(particles));

    % r in range [0, 1/N]
    r = rand() / N;

    c = weights(1);
    i = 1;

    for n = 1:N
        
        u = r + (n - 1) / N;

        while u > c
            i = i + 1;
            c = c + weights(i);
        end

        noise_vector = [0.05, 0.05, 0.01]; 
        new_particles(n, :) = particles(i, :) + randn(1, 3) .* noise_vector;

        new_particles(n, 3) = wrapTo2Pi(new_particles(n, 3)); %theta in ragne [0, 2*pi]
    end

end

