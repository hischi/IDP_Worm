function [ dH_dq_ ] = dH_dq( q, j, i )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

xi = ceil(i/3);
if xi ~= j
    dH_dq_ = zeros(4);
else

    if mod(i, 3) == 1
        dH_dq_ = dH_dq1(q(3*j-2:3*j));
    elseif mod(i, 3) == 2
        dH_dq_ = dH_dq2(q(3*j-2:3*j));
    else
        dH_dq_ = dH_dq3(q(3*j-2:3*j));
    end
end
end

