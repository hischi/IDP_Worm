function [ dHglob_ ] = dwHj_dq( q, j, x )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

i = ceil(x/3);
k = mod(x-1, 3) + 1;

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
    dHglob_ = zeros(4);
else
    dHglob_ = wHj(q, i-1) * (H_upper * dH_dq(q, i, x) * H_lower) * iHj(q, i, j);
end

end

