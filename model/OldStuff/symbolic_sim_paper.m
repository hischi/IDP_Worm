syms q1 dq1 ddq1 q2 dq2 ddq2 q3 dq3 ddq3 p1 p2 p3

assume(q1, 'real');
assume(q2, 'real');
assume(q3, 'real');
assume(dq1, 'real');
assume(dq2, 'real');
assume(dq3, 'real');
assume(ddq1, 'real');
assume(ddq2, 'real');
assume(ddq3, 'real');

assume(p1, 'real');
assume(p2, 'real');
assume(p3, 'real');

% generalised coords.
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];
p = [p1; p2; p3];

m = 0.15;               % mass
g = 9.81 * [0; 0; 1; 0];% gravity constant
r_B = 0.025;            % distance between muscle-center and ee-center
r_B1 = [-r_B; 0; 0; 1];
r_B2 = [sqrt(r_B); sqrt(r_B); 0; 1];
r_B3 = [sqrt(r_B); -sqrt(r_B); 0; 1];
r_SXi = [0; 0; 0; 1];   % vector between ee-center and center-of-mass
M_rbend = 0;            % bending momentum
F_eS = [0; 0; 0; 0];    % external forces

%% forces
F_eBspr = 7.944 * (100*q).^2 - 195.3 * (100*q) + 1264;
F_eBdmp = 10 * dq;

%% kinematics
l = 1/3 * (q1+q2+q3);
kappa = 2 * sqrt(q1^2+q2^2+q3^2-q1*q2-q2*q3-q3*q1) / ((q1+q2+q3) * r_B);
phi = atan(sqrt(3)*(q3-q2) / (q2+q3-2*q1));

c_phi = cos(phi);
c_kl = cos(kappa*l);
s_phi = sin(phi);
s_kl = sin(kappa*l);

% H = [c_phi^2*(c_kl-1)+1,    s_phi*c_phi*(c_kl-1),   c_phi*s_kl,     c_phi*(1-c_kl)/kappa; ...
%      c_phi*s_phi*(c_kl-1),  c_phi^2*(1-c_kl)+c_kl,  s_phi*s_kl,     s_phi*(1-c_kl)/kappa; ...
%      -c_phi*s_kl,           -s_phi*s_kl,            c_kl,           s_kl/kappa; ...
%      0,                     0,                      0,              1];
%  
H = [1, 0, 0, 0;...
     0, 1, 0, 0;...
     0, 0, 1, l;...
     0, 0, 0, 1];

 
%% mass matrix M
M1_1 = m * (diff(H, q(1)) * r_SXi)' * diff(H, q(1)) * r_SXi;
M1_2 = m * (diff(H, q(2)) * r_SXi)' * diff(H, q(1)) * r_SXi;
M1_3 = m * (diff(H, q(3)) * r_SXi)' * diff(H, q(1)) * r_SXi;

M2_1 = m * (diff(H, q(1)) * r_SXi)' * diff(H, q(2)) * r_SXi;
M2_2 = m * (diff(H, q(2)) * r_SXi)' * diff(H, q(2)) * r_SXi;
M2_3 = m * (diff(H, q(3)) * r_SXi)' * diff(H, q(2)) * r_SXi;

M3_1 = m * (diff(H, q(1)) * r_SXi)' * diff(H, q(3)) * r_SXi;
M3_2 = m * (diff(H, q(2)) * r_SXi)' * diff(H, q(3)) * r_SXi;
M3_3 = m * (diff(H, q(3)) * r_SXi)' * diff(H, q(3)) * r_SXi;

M = [M1_1 M1_2 M1_3;...
     M2_1 M2_2 M2_3;...
     M3_1 M3_2 M3_3];     

%% homogeneous force vector N
F_eB1p = - H * (F_eBspr(1) - F_eBdmp(1)) * [0; 0; 1; 0];
F_eB2p = - H * (F_eBspr(2) - F_eBdmp(2)) * [0; 0; 1; 0];
F_eB3p = - H * (F_eBspr(3) - F_eBdmp(3)) * [0; 0; 1; 0];

N1 = diff(M_rbend, q(1)) - (m * g') * diff(H, q(1)) * r_SXi - ...
     F_eB1p' * diff(H, q(1)) * r_B1 - ...
     F_eB2p' * diff(H, q(1)) * r_B2 - ...
     F_eB3p' * diff(H, q(1)) * r_B3;
 
N2 = diff(M_rbend, q(2)) - (m * g') * diff(H, q(2)) * r_SXi - ...
     F_eB1p' * diff(H, q(2)) * r_B1 - ...
     F_eB2p' * diff(H, q(2)) * r_B2 - ...
     F_eB3p' * diff(H, q(2)) * r_B3;
 
N3 = diff(M_rbend, q(3)) - (m * g') * diff(H, q(3)) * r_SXi - ...
     F_eB1p' * diff(H, q(3)) * r_B1 - ...
     F_eB2p' * diff(H, q(3)) * r_B2 - ...
     F_eB3p' * diff(H, q(3)) * r_B3;
 
N = [N1; N2; N3];
 
%% pressure-depended in-homogenous force vector
p_fact1 = 83.33+(q1-0.16) * 1910;
F1 = (400/((0.16-q1)*80+1)-150-3*p_fact1) + p1 * p_fact1;

p_fact2 = 83.33+(q2-0.16) * 1910;
F2 = (400/((0.16-q2)*80+1)-150-3*p_fact2) + p2 * p_fact2;

p_fact3 = 83.33+(q3-0.16) * 1910;
F3 = (400/((0.16-q3)*80+1)-150-3*p_fact3) + p3 * p_fact3;

tau1 =  (F1 * [0;0;1;0])'* H' * diff(H, q(1)) * r_B1 + ...
        (F2 * [0;0;1;0])'* H' * diff(H, q(1)) * r_B2 + ...
        (F3 * [0;0;1;0])'* H' * diff(H, q(1)) * r_B3;
    
tau2 =  (F1 * [0;0;1;0])'* H' * diff(H, q(2)) * r_B1 + ...
        (F2 * [0;0;1;0])'* H' * diff(H, q(2)) * r_B2 + ...
        (F3 * [0;0;1;0])'* H' * diff(H, q(2)) * r_B3;
    
tau3 =  (F1 * [0;0;1;0])'* H' * diff(H, q(3)) * r_B1 + ...
        (F2 * [0;0;1;0])'* H' * diff(H, q(3)) * r_B2 + ...
        (F3 * [0;0;1;0])'* H' * diff(H, q(3)) * r_B3; 
    
tau = [tau1; tau2; tau3];

%%
q1 = 0.16;
q2 = 0.14;
q3 = 0.14;

dq1 = 0;
dq2 = 0;
dq3 = 0;

p1 = 1;
p2 = 1;
p3 = 1;

lm = double(subs(l));
kappam = double(subs(kappa));
dkappam = double(subs(diff(kappa, q(2))));
phim = double(subs(phi));
dphim = double(subs(diff(phi, q(2))));

Hm = double(subs(H));
dHm = double(subs(diff(H, q(1))));

Mm = double(subs(M));

Fspr = double(subs(F_eBspr));
F_eB1pm = double(subs(F_eB1p));
F_eB2pm = double(subs(F_eB2p));
F_eB3pm = double(subs(F_eB3p));

Nm = double(subs(N));

p_fact1m = double(subs(p_fact1));
F1m = double(subs(F1));

p_fact2m = double(subs(p_fact2));
F2m = double(subs(F2));

p_fact3m = double(subs(p_fact3));
F3m = double(subs(F3));

taum = double(subs(tau));

ddq = inv(Mm) * (taum - Nm);
