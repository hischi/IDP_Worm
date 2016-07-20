%% Kalman Filter init
Tsample = 0.01;
% System Model
Ac =[0 1 0; 0 0 1; 0 0 0];
Bc =[0;0;0];
Cc =[1 0 0];
Dc = 0;
Sysc = ss(Ac,Bc,Cc,Dc);
Sysd = c2d(Sysc,Tsample);
Ad = Sysd.a;
Bd = Sysd.b;
Cd = Sysd.c;
Dd = Sysd.d;

G = [0;1;1];
quant = 4*1e-4;
R = quant^2/3;

%Parameters
c = 0.000001; %best accuracy (0.025), peak vel error at 0.4
%c = 0.0001; % accuracy (0.05), peak vel error at 0.4
%c = 0.001; % accuracy (0.1), best peak vel error at 0.4
%c = 0.01; % accuracy (0.15), best peak vel error at 0.6

Q = c; %covariance of unbiased white noise (ddq surrogate)
M = dlqe(Ad,G,Cd,Q,R); %Kalman Gain
[Ae,Be,Ce,De]=destim(Ad,Bd,Cd,Dd,M);

clear Ac Ad Bc Bd c Cc Cd d Dc Dd G M Q quant R Sysc Sysd