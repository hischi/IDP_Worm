function [ N ] = N( q, m_lower, m_upper, k, ls )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

N = zeros(size(q));
n = ceil(length(q)/3);

F_spr = k * (q - ls);
if n > 1
    F_spr(1:3*n-3) = F_spr(1:3*n-3) - F_spr(4:3*n);
end

for r = 1:length(q)
    Gall = 0;
    Fall = 0;
    for xi = 1:n
        G = 0;
        F = 0;
        
        if xi == n
            m = m_lower;
        else
            m = m_lower + m_upper;
        end
        
        G = (m * [0;0;9.81;0]') * dwHj_dq(q, xi, r) * [0;0;0;1];
        
        F =   - (F_spr(3*xi-2) * [0;0;1;0]') * wHj(q, xi)' * dwHj_dq(q, xi, r) * [0.025;0;0;1];
        F = F - (F_spr(3*xi-1) * [0;0;1;0]') * wHj(q, xi)' * dwHj_dq(q, xi, r) * [-0.013;0.021354;0;1];
        F = F - (F_spr(3*xi)   * [0;0;1;0]') * wHj(q, xi)' * dwHj_dq(q, xi, r) * [-0.013;-0.021354;0;1];
        
        Gall = Gall + G;
        Fall = Fall + F;
    end    
    N(r) = Gall + Fall;
end

%N = N - F_spr;

end

