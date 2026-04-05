function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here


% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    %public_vars = init_particle_filter(read_only_vars, public_vars);
    %public_vars = init_kalman_filter(read_only_vars, public_vars);

    public_vars.lidar_log = [];
    public_vars.gnss_log = [];
    public_vars.motion_vector = [0, 0];

end

%% Tasks 2-4 assumes different map, start position in setup and
% start_position = [10, 10, pi/2]; % (x, y, theta)
% map_name = 'maps/std_map.txt';

% Comment this for task 2-4
public_vars = plan_motion(read_only_vars, public_vars);


public_vars.lidar_log = [public_vars.lidar_log; read_only_vars.lidar_distances];
public_vars.gnss_log = [public_vars.gnss_log; read_only_vars.gnss_position(:)'];

if read_only_vars.counter == 500
    % Task 2
    lidar_std = std(public_vars.lidar_log);
    gnss_std = std(public_vars.gnss_log);
    disp('LiDAR Standard Deviations (8 channels):'); disp(lidar_std);
    disp('GNSS Standard Deviations (X, Y):'); disp(gnss_std);
    
    % Plot Histograms
    figure(2);
    theoretical_dist = [1, 2, 3, 4, 1, 6, 10, 8];
    for i = 1:8
        nexttile;
        histogram(public_vars.lidar_log(:, i), 15);
        title(sprintf('Channel %d (%.0f m)', i, theoretical_dist(i)), 'FontSize', 12);
        xlabel('Measured Distance', 'FontSize', 10);
        ylabel('Frequency', 'FontSize', 10);
        grid on;
    end
    
    theoretical_pos = ['X', 'Y'];
    for i = 1:2
        nexttile;
        histogram(public_vars.gnss_log(:, i), 15);
        title(sprintf('GNSS %s position', theoretical_pos(i)), 'FontSize', 12);
        xlabel('Measured position', 'FontSize', 10);
        ylabel('Frequency', 'FontSize', 10);
        grid on;
    end

    % Task 3
    lidar_cov = cov(public_vars.lidar_log);
    gnss_cov = cov(public_vars.gnss_log);
    
    disp('LiDAR Covariance Matrix (8x8):'); disp(lidar_cov);
    disp('GNSS Covariance Matrix (2x2):'); disp(gnss_cov);

    lidar_diag = diag(lidar_cov)';
    lidar_variance = lidar_std.^2;
    disp('LiDAR Diagonal of Covariance:');
    disp(lidar_diag);
    disp('LiDAR std^2 : ');
    disp(lidar_variance);

    gnss_diag = diag(gnss_cov)';
    gnss_variance = gnss_std.^2;
    disp('GNSS Diagonal of Covariance: ');
    disp(gnss_diag);
    disp('GNSS std^2: ');
    disp(gnss_variance);

    if all(abs(lidar_diag - lidar_variance) < 1e-10) && all(abs(gnss_diag - gnss_variance) < 1e-10)
        disp('VERIFICATION SUCCESS: std^2 == diagonal of covariance matrix');
    else
        disp('VERIFICATION FAILED: Diagonal values doesnt match');
    end



    % Task 4
    sigma_lidar_ch1 = lidar_std(1); 
    sigma_gnss_x = gnss_std(1);
    
    % X axis
    x_axis = -2:0.005:2;
    
    
    pdf_lidar = norm_pdf(x_axis, 0, sigma_lidar_ch1);
    pdf_gnss = norm_pdf(x_axis, 0, sigma_gnss_x);

    figure(3);
    hold on; grid on;
    plot(x_axis, pdf_lidar, 'r', 'LineWidth', 2);
    plot(x_axis, pdf_gnss, 'b', 'LineWidth', 2);
    title('Task 4: PDF sensor noise (\mu = 0)', 'FontSize', 12);
    xlabel('Measurement err', 'FontSize', 10);
    ylabel('PDF', 'FontSize', 10);
    legend('LiDAR (channel 1)', 'GNSS (X axis)')

end




% % 9. Update particle filter
% public_vars.particles = update_particle_filter(read_only_vars, public_vars);
% 
% % 10. Update Kalman filter
% [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
% 
% 11. Estimate current robot position
%public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
public_vars.estimated_pose = read_only_vars.mocap_pose; % for week 2 assignment , we can use MoCap position
% 
% % 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 
% 13. Plan next motion command
%public_vars = plan_motion(read_only_vars, public_vars);



end

