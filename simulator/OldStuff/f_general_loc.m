function [ R_ih ] = f_general_loc( k, i, R_ib )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    
    R_ih = H_i(k, i) * R_ib;
end
