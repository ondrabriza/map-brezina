function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here
mu = mu(:);
z = z(:);
K = sigma * kf.C' * inv(kf.C * sigma * kf.C' + kf.Q);
new_mu = mu + K * (z - kf.C * mu);

B = K*kf.C;

new_mu = new_mu(:)';
new_sigma = (eye(size(B))-B)*sigma;

end

