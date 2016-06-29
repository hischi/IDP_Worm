function [ H_i ] = H_i( k, i )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

    if k.kappa(i) == 0
        H_i = [ 1, 0, 0, 0;...
                0, 1, 0, 0;...
                0, 0, 1, k.l(i);...
                0, 0, 0, 1];
    else
        c_phi = cos(k.phi(i));
        s_phi = sin(k.phi(i));
        c_kl = cos(k.kappa(i)*k.l(i));
        s_kl = sin(k.kappa(i)*k.l(i));

        H_i = [c_phi^2*(c_kl-1)+1, s_phi*c_phi*(c_kl-1), c_phi*s_phi, c_phi*(1-c_kl)/k.kappa(i); ...
             c_phi*s_phi*(c_kl-1), c_phi^2*(c_kl-1)+c_kl, s_phi*s_kl, s_phi*(1-c_kl)/k.kappa(i); ...
             -c_phi*s_kl, -s_phi*s_kl, c_kl, s_kl/k.kappa(i); ...
             0, 0, 0, 1];
    end

end

