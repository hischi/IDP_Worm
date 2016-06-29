function [ H_wi ] = H_wi( k, i )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

H_wi = eye(4);
for xi = 1:i
    H_wi = H_wi * H_i(k, xi);
end

end

