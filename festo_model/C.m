function [ C ] = C( q, dq, m_lower, m_upper )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

C = zeros(length(q));
n = ceil(length(q)/3);

for r = 1:length(q)
    for c = 1:length(q)
        section = ceil(max(r,c)/3);
        
        for xi = section:n
            if xi == n
                m = m_lower;
            else
                m = m_lower + m_upper;
            end
            
            C(r,c) = C(r,c) + m * (ddwHj_dqdt(q, dq, xi, c) * [0;0;0;1])' * (dwHj_dq(q, xi, r) * [0;0;0;1]);

        end
    end        
end

end