function [ k ] = f_specific( q, r_B )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

k.phi = atan2(sqrt(3)*(q(:,3)-q(:,2)), q(:, 2) + q(:, 3) - 2 * q(:, 1));
k.kappa = 2*sqrt(q(:,1)^2 + q(:,2)^2 + q(:,3)^2 - q(:,1)*q(:,2) - q(:,2)*q(:,3) - q(:,3)*q(:,1)) / ((q(:,1)+q(:,2)+q(:,3))*r_B);
k.l = (q(:,1)+q(:,2)+q(:,3)) / 3;

end

