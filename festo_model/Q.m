function [ Q ] = Q( F, q, dq, d )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

%Q = F - d * dq;

Q = zeros(size(q));
n = ceil(length(q)/3);

if n > 1
    F(1:3*n-3) = F(1:3*n-3) - F(4:3*n);
end

for r = 1:length(q)
    for xi = 1:n
        
        Q(r) = Q(r) + (F(3*xi-2) * [0;0;1;0]') * wHj(q, xi)' * dwHj_dq(q, xi, r) * [0.025;0;0;1];
        Q(r) = Q(r) + (F(3*xi-1) * [0;0;1;0]') * wHj(q, xi)' * dwHj_dq(q, xi, r) * [-0.013;0.021354;0;1];
        Q(r) = Q(r) + (F(3*xi)   * [0;0;1;0]') * wHj(q, xi)' * dwHj_dq(q, xi, r) * [-0.013;-0.021354;0;1];

    end    
end


Q = Q - d * dq;

end