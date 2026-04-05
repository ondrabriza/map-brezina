function [path] = sine_path(start_point, end_point, amplitude, frequency, number_of_points)
    % start_point: [x, y] starting point of the base axis
    % end_point: [x, y] ending point of the base axis
    % amplitude: peak deviation from the base axis 
    % frequency: number of full wave cycles along the path
    % number_of_points: total number of waypoints
    
    % Generate the base straight line
    x_base = linspace(start_point(1), end_point(1), number_of_points)';
    y_base = linspace(start_point(2), end_point(2), number_of_points)';
    
    % Calculate the direction and length of the axis
    dx = end_point(1) - start_point(1);
    dy = end_point(2) - start_point(2);
    line_length = norm([dx, dy]);
    
    % Prevent division by zero if start and end points are the same
    if line_length == 0
        path = [x_base, y_base];
        return;
    end
    
    % Calculate the perpendicular (normal) vector for the wave offset
    perp_x = -dy / line_length;
    perp_y = dx / line_length;
    
    % Generate the sine wave offset
    t = linspace(0, 1, number_of_points)';
    offset = amplitude * sin(2 * pi * frequency * t);
    
    % Add the perpendicular offset to the base line
    x = x_base + offset * perp_x;
    y = y_base + offset * perp_y;
    
    path = [x, y];
end