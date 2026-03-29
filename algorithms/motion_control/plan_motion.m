function [public_vars] = plan_motion(read_only_vars, public_vars)
    %PLAN_MOTION Summary of this function goes here
    
    % % I. Pick navigation target
    % target = get_target(public_vars.estimated_pose, public_vars.path);
    % % II. Compute motion vector
    % public_vars.motion_vector = [0.5,0.5];
    
    % Task 5
    
    t = read_only_vars.counter;
    % Assumes spawn position [1, 1, pi/2];

    % 1. Drive STRAIGHT (cca 150 cycles)
    if t < 150
        public_vars.motion_vector = [0.5, 0.5]; 
        
    % 2. Turn RIGHT
    elseif t < 166
        public_vars.motion_vector = [-0.1, 0.1]; 
        
    % 3. Drive STRAIGHT
    elseif t < 256
        public_vars.motion_vector = [0.5, 0.5];
        
    % 4. Turn RIGHT
    elseif t < 272
        public_vars.motion_vector = [-0.1, 0.1];
        
    % 5. Drive STRAIGHT
    elseif t < 412
        public_vars.motion_vector = [0.5, 0.5];
        
    % 6. Turn LEFT
    elseif t < 428
        public_vars.motion_vector = [0.1, -0.1];
        
    % 7. Drive STRAIGHT 
    elseif t < 498
        public_vars.motion_vector = [0.5, 0.5];
        
    % 8. Turn LEFT
    elseif t < 514
        public_vars.motion_vector = [0.1, -0.1];
        
    % 9. Drive STRAIGHT and slightly RIGHT
    elseif t > 514
        public_vars.motion_vector = [0.493, 0.5];
    
    end
end