function [ H_ ] = wHj( q, j )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

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
      
       
H_ = H_base;
for xi = 1:j
    H_ = H_ * (H_upper * H(q(3*xi-2:3*xi)) * H_lower);
end

end
