function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, dt)
%EKF_PREDICT Summary of this function goes here

x_old = mu(1);
y_old = mu(2);
theta_old = mu(3);
v = u(1);
w = u(2);

x = x_old + cos(theta_old)*v*dt;
y = y_old + sin(theta_old)*v*dt;
theta = theta_old + w*dt;

new_mu = [x, y, theta];

G = [1 0 -sin(theta_old) * v * dt;
     0 1 cos(theta_old) * v * dt;
     0 0 1];
new_sigma = G * sigma * G' + kf.R;

end

