% Derive DE for robot
% options:
save = 1;       % save to matlab files
equalCase = 1;  % all lengths are equal


%% declare symbolics
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 Fm1 Fm2 Fm3 g
assume(q1 > 0);     % muscle length [m]
assume(q2 > 0);     % muscle length [m]
assume(q3 > 0);     % muscle length [m]
assume(dq1, 'real');    % muscle length velocity [m/s]
assume(dq2, 'real');    % muscle length velocity [m/s]
assume(dq3, 'real');    % muscle length velocity [m/s]
assume(ddq1, 'real');   % muscle length acceleration [m/s^2]
assume(ddq2, 'real');   % muscle length acceleration [m/s^2]
assume(ddq3, 'real');   % muscle length acceleration [m/s^2]

assume(Fm1, 'real');    % muscle force (active - spring force of muscle) [N]
assume(Fm2, 'real');    % muscle force (active - spring force of muscle) [N]
assume(Fm3, 'real');    % muscle force (active - spring force of muscle) [N]

assume(g, 'real');      % gravity acceleration [m/s^2]


%% constants
load('RobotConstants');
% m:    mass of end-effector [kg]
% r_B:  distance center to mid of muscle [m]
% k:    spring constant (metallic spring) [N/m]
% d:    damping constant (overall-system) [Ns/m]
% l0:   norm. length of muscle [m]
% ls:   norm. length of spring [m]

%% help relations
q = [q1; q2; q3];           % muscle lengths [m]
dq = [dq1; dq2; dq3];       % muscle length velocities [m/s]
ddq = [ddq1; ddq2; ddq3];   % muscle length accelerations [m/s^2]
Fm = [Fm1; Fm2; Fm3];       % muscle force (active - spring) [N] 
h = l0 - q;                 % absolute contraction [m]
hrel = h / l0;              % relative contraction
hs = ls - q;                % absolute spring contraction [m]
hsrel = hs / ls;            % relative spring contraction

%% kinematics (from Festo paper)
l = 1/3 * (q1+q2+q3);   
kappa = 2 * sqrt(q1^2+q2^2+q3^2-q1*q2-q2*q3-q3*q1) / ((q1+q2+q3) * r_B);
phi = atan(sqrt(3)*(q3-q2)/ (q2+q3-2*q1));

c_phi = cos(phi);
c_kl = cos(kappa*l);
s_phi = sin(phi);
s_kl = sin(kappa*l);

if equalCase == 0
    r = [c_phi*(1-c_kl)/kappa;...   % position of end-effector   
         s_phi*(1-c_kl)/kappa;...
         -s_kl/kappa];              
else
    r = [0;...  % position of end-effector if all lengths are equal
         0;...
         -l];
end
 
%% potential energies
Vh = m * g * [0 0 1]*r; % height energy
Vs = 1/2 * k * hs.^2;    % spring energy (metallic spring)
V = Vh + sum(Vs);
 
%% kinematic energies
drdt = jacobian(r) * dq;        % velocity of end-effector
T = 1/2 * m * (drdt' * drdt);   % translational kinetic energy
 
%% generalized forces
Fdmp = -d * dq; % damping force
Q = Fm + Fdmp;  % I may have to consider the coordinate of the end-effector here...

%% Euler-Lagrange equation
L = T - V;

%    | d/dt          (total time deriv.)|-|   dL/dq
%             |  dL/d(dq)    |
LG = jacobian(gradient(L, dq), dq) * ddq - gradient(L, q);

%% N vector
N = subs(LG,{ddq1, ddq2, ddq3,g},{0,0,0,9.81});

%% M matrix
% first column
M1 = subs(LG,{ddq1, ddq2, ddq3,g},{1,0,0,9.81}) - N;
% second column
M2 = subs(LG,{ddq1, ddq2, ddq3,g},{0,1,0,9.81}) - N;
% third column
M3 = subs(LG,{ddq1, ddq2, ddq3,g},{0,0,1,9.81}) - N;

M = [M1, M2, M3];

if equalCase ~= 0
    M = 3 * M .* eye(size(M));  % correct matrix (it should be invertible)
end

%% save results
if save ~= 0
    if equalCase == 0
        disp('N function');
        Nfunc = matlabFunction(N,'File','Nfunc','Vars',{q; dq});
        disp('M function');
        Mfunc = matlabFunction(M,'File','Mfunc','Vars',{q});
        disp('Q function');
        Qfunc = matlabFunction(Q,'File','Qfunc','Vars',{dq; Fm});
    else
        Nfunc2 = matlabFunction(N,'File','Nfunc2','Vars',{q; dq});
        Mfunc2 = matlabFunction(M,'File','Mfunc2','Vars',{q});
        Qfunc2 = matlabFunction(Q,'File','Qfunc2','Vars',{dq; Fm});
    end
end

