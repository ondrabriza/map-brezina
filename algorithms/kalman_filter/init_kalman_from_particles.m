function [public_vars] = init_kalman_from_particles(public_vars, mu, sigma )
%INIT_KALMAN_FILTER Summary of this function goes here
    if isempty(public_vars.kf.C)
        public_vars.kf.C = [1, 0, 0;
                            0, 1, 0];
    end

    if isempty(public_vars.kf.Q)
        public_vars.kf.Q = [0.25, 0.03;
                            0.03, 0.25];
    end

    if isempty(public_vars.kf.R)
        public_vars.kf.R = diag([0.0005, 0.0005, 0.0001]);
    end
    
    public_vars.mu = mu;
    public_vars.sigma = sigma;

end

