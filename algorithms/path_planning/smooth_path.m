function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

    % if empty or straight of 2 points, nothing to smooth
    if isempty(old_path) || size(old_path, 1) < 3
        new_path = old_path;
        return;
    end

    % Smoothing parameters
    alpha = 0.2;   
    beta = 0.8;    
    
    % number of iterations
    num_of_iterations = 5; 

    %init
    new_path = old_path; 
    N = size(old_path, 1);

    for k = 1:num_of_iterations
        for i = 2:(N-1)

            % y_i = y_i + alpha*(x_i - y_i) + beta*(y_{i-1} + y_{i+1} - 2*y_i)
            
            a = alpha * (old_path(i, :) - new_path(i, :));
            b = beta * (new_path(i-1, :) + new_path(i+1, :) - 2 * new_path(i, :));
            
            new_path(i, :) = new_path(i, :) + a + b;
        end
    end
end