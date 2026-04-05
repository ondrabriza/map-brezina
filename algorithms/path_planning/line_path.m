function [path] = line_path(start_point, end_point, number_of_points)
    % start_point: [x, y] coordinates of the starting point
    % end_point: [x, y] coordinates of the target point
    % number_of_points: total number of waypoints
    
    x = linspace(start_point(1), end_point(1), number_of_points)';
    y = linspace(start_point(2), end_point(2), number_of_points)';
    
    path = [x, y];
end