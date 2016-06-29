function [ R_ih ] = f_general_glob( k, i, R_w )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

R_ih = H_wi(k, i) * R_w;

end

