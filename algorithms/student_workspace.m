function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

%determine if robot is indoor or outdoor
gnss_valid = is_gnss_valid(read_only_vars.gnss_position); 
if gnss_valid
    public_vars.kf_enabled = 1;
    gnss_history = rmmissing(read_only_vars.gnss_history);
else
    public_vars.pf_enabled = 1;
    gnss_history =[];


end

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    public_vars.kf_enabled = 0;
    public_vars.pf_enabled = 0;
    public_vars.motion_vector = [-0.05,0.05];
    public_vars.pf_initialized = 0;
    public_vars.kf_initialized = 0;
    public_vars.injection_rate = 0.05;
    public_vars.mu = [];
    public_vars.sigma = [];
    public_vars.kf.Q = [];
    public_vars.kf.C = [];
    public_vars.kf.R = [];

end


% indoor  - init particles
if ~public_vars.pf_initialized && ~gnss_valid
    public_vars.particles = init_particle_filter(read_only_vars, public_vars, read_only_vars.max_particles);
    public_vars.pf_initialized = 1;
end

% outdoor start - init kalman after 10 cycles
if size(gnss_history,1) > 10 && ~public_vars.kf_initialized && gnss_valid
    public_vars = init_kalman_filter(read_only_vars, public_vars);
    public_vars.kf_initialized = 1;
end

% switch from indoor to outdoor
if public_vars.pf_enabled && public_vars.kf_enabled && gnss_valid 
    [pf_mu, pf_sigma] = estimate_particle_pose(public_vars, read_only_vars);
    public_vars = init_kalman_from_particles(public_vars, pf_mu, pf_sigma);
    public_vars.kf_initialized = 1;

    public_vars.pf_enabled = 0;
end

% switch from outdoor to indoor
if public_vars.pf_enabled && public_vars.kf_enabled && ~gnss_valid 
    public_vars.particles = init_particles_from_kalman(public_vars, read_only_vars);
    public_vars.kf_enabled = 0; 
end


% 9. Update particle filter
if public_vars.pf_enabled && public_vars.pf_initialized
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
end

% 10. Update Kalman filter
if public_vars.kf_enabled && public_vars.kf_initialized
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
end

% 11. Estimate current robot position
if ~isempty(public_vars.mu) && ~isempty(public_vars.sigma) || ~isempty(public_vars.particles)
    public_vars.estimated_pose = estimate_pose(public_vars, read_only_vars); % (x,y,theta)
end

% 12. Path planning
if isempty(public_vars.path) && ~isempty(public_vars.estimated_pose)
    public_vars.path = plan_path(read_only_vars, public_vars);
    
end

% 13. Plan next motion command
if ~isempty(public_vars.estimated_pose) && ~isempty(public_vars.path)
    public_vars = plan_motion(read_only_vars, public_vars);
end


end
