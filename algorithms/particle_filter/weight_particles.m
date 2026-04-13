function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here
  
    % set sigma of normal distribution
    sigma = 0.15; 
    
    differences = particle_measurements - lidar_distances;
    
    % calc probabilities/weights 
    probabilities = norm_pdf(differences, 0, sigma); % using normal distribution func we wrote in previous week
    weights = prod(probabilities, 2);
    weights(isnan(weights)) = 0;
    summ = sum(weights);

    if summ == 0
        N = size(weights, 1);
        weights = ones(N, 1) / N;
    else
        weights = weights / summ;
    end

end

