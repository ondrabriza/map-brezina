function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here
    gnss_history = rmmissing(read_only_vars.gnss_history);
    gnss_mu = mean(gnss_history);
    gnss_cov = cov(gnss_history);

    public_vars.kf.C = [1, 0, 0;
                        0, 1, 0];

    public_vars.kf.Q = gnss_cov ;
    disp(public_vars.kf.Q)
    public_vars.kf.R = diag([0.0005, 0.0005, 0.0001]);
    % public_vars.mu = [2,2,pi/2];
    % public_vars.sigma = zeros(3,3);

    
    public_vars.mu = [gnss_mu, pi/2];
    public_vars.sigma = [gnss_cov(1,1),gnss_cov(1,2), 0;
                         gnss_cov(2,1),gnss_cov(2,2),0;
                         0,0, 2*pi];

end

