function [ ddwHj_ddq_ ] = ddwHj_ddq( q, j, x, y )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

alpha = ceil(x/3);
beta = mod(x-1, 3) + 1;

i = ceil(y/3);
k = mod(y-1, 3) + 1;

H_base = [1, 0, 0,  0;...
          0, 1, 0,  0;...
          0, 0, 1,  0;...
          0, 0, 0,  1];  

H_upper = [1, 0, 0, 0;...
           0, 1, 0, 0;...
           0, 0, 1, 0.051;...
           0, 0, 0, 1];
     
H_lower = [1, 0, 0, 0;...
           0, 1, 0, 0;...
           0, 0, 1, 0.041;...
           0, 0, 0, 1];      

if i > j
    ddwHj_ddq_ = zeros(4);
    
elseif alpha < i && i <= j
    ddwHj_ddq_ = wHj(q, alpha-1) * (H_upper * dH_dq(q, alpha, x) * H_lower) * iHj(q, alpha, i) * (H_upper * dH_dq(q, i, y) * H_lower) * iHj(q, i, j);
    
else
    ddwHj_ddq_ = wHj(q, i-1) * (H_upper * ddH_ddq(q, i, x, y) * H_lower) * iHj(q, i, j);
end

end

