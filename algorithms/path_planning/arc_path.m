function [path] = arc_path(center_point, radius, start_angle, end_angle, number_of_points)
    % center_point: [x, y] coordinates of the arc's center
    % radius: radius of the arc in meters
    % start_angle: starting angle in radians (e.g., 0)
    % end_angle: ending angle in radians (e.g., pi for a semi-circle)
    % number_of_points: total number of waypoints
    
    theta = linspace(start_angle, end_angle, number_of_points)';
    
    x = center_point(1) + radius * cos(theta);
    y = center_point(2) + radius * sin(theta);
    
    path = [x, y];
end