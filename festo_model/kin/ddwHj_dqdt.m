function [ ddwHj_dqdt_ ] = ddwHj_dqdt( q, dq, j, x )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

i = ceil(x/3);
k = mod(x-1, 3) + 1;

if i > j
    ddwHj_dqdt_ = zeros(4);
    
else
   ddwHj_dqdt_ = zeros(4);
   for xi = 1:j
       ddwHj_dqdt_ = ddwHj_dqdt_ + ddwHj_ddq(q, j, 3*xi-2, x) * dq(3*xi-2);
       ddwHj_dqdt_ = ddwHj_dqdt_ + ddwHj_ddq(q, j, 3*xi-1, x) * dq(3*xi-1);
       ddwHj_dqdt_ = ddwHj_dqdt_ + ddwHj_ddq(q, j, 3*xi, x) * dq(3*xi);
   end    
end

end

