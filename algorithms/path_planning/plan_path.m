function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

planning_required = 0;

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    number_of_points = 50;
    path = line_path([2,15],[5,25],number_of_points);
    path = [path; line_path([5,25],[10,15],number_of_points)];
    path = [path; arc_path([10,12.5], 2.5 ,pi/2,-pi/2,number_of_points)];
    path = [path; arc_path([10,7.5], 2.5 ,pi/2,3*pi/2,number_of_points)];
    path = [path; sine_path([10,5], [28,15], 2, 10, 3*number_of_points)];

    % remove duplicate points
    valid_points = [true; any(diff(path) ~= 0, 2)];
    path = path(valid_points, :);
    
end

end

