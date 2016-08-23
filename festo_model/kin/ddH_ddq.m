function [ ddH_ddq_ ] = ddH_ddq( q, j, x, y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

alpha = ceil(x/3);
beta = mod(x-1, 3) + 1;

i = ceil(y/3);
k = mod(y-1, 3) + 1;

if alpha ~= j || i ~= j
    ddH_ddq_ = zeros(4);
else

    if beta == 1
        if k == 1
            ddH_ddq_ = ddH_dq1dq1(q(3*j-2:3*j));
        elseif k == 2
            ddH_ddq_ = ddH_dq1dq2(q(3*j-2:3*j));
        else
            ddH_ddq_ = ddH_dq1dq3(q(3*j-2:3*j));
        end
    elseif beta == 2
        if k == 1
            ddH_ddq_ = ddH_dq2dq1(q(3*j-2:3*j));
        elseif k == 2
            ddH_ddq_ = ddH_dq2dq2(q(3*j-2:3*j));
        else
            ddH_ddq_ = ddH_dq2dq3(q(3*j-2:3*j));
        end
    else
        if k == 1
            ddH_ddq_ = ddH_dq3dq1(q(3*j-2:3*j));
        elseif k == 2
            ddH_ddq_ = ddH_dq3dq2(q(3*j-2:3*j));
        else
            ddH_ddq_ = ddH_dq3dq3(q(3*j-2:3*j));
        end
    end
end

end

