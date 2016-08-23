% Derive DE for robot
% options:
save = 1;       % save to matlab files
sections = 2;


%% declare symbolics
syms g
q = sym('q',[(3*sections) 1]);

for i=1:(3*sections)
    assume(q(i) > 0);     % muscle length [m]
end



%% constants
load('RobotConstants');
% m:    mass of end-effector [kg]
% r_B:  distance center to mid of muscle [m]
% k:    spring constant (metallic spring) [N/m]
% d:    damping constant (overall-system) [Ns/m]
% l0:   norm. length of muscle [m]
% ls:   norm. length of spring [m]

%% help relations
r_S = [0;0;0;1];

%% kinematics (from Festo paper)
q_loc = sym('q_loc',[3 1]);
k_loc = sym('k_loc',[3 1]);

f_loc(q_loc) = [1/3 * (q_loc(1)+q_loc(2)+q_loc(3)); ... % l; kappa; phi
                2 * sqrt(q_loc(1)^2+q_loc(2)^2+q_loc(3)^2-q_loc(1)*q_loc(2)-q_loc(2)*q_loc(3)-q_loc(3)*q_loc(1)) / ((q_loc(1)+q_loc(2)+q_loc(3)) * r_B); ...
                atan2(sqrt(3)*(q_loc(3)-q_loc(2)), (q_loc(2)+q_loc(3)-2*q_loc(1)))];

H_loc(k_loc) = ...
    [cos(k_loc(3))^2*(cos(k_loc(2)*k_loc(1))-1)+1,              sin(k_loc(3))*cos(k_loc(3))*(cos(k_loc(2)*k_loc(1))-1),             cos(k_loc(3))*sin(k_loc(2)*k_loc(1)),   cos(k_loc(3))*(1-cos(k_loc(2)*k_loc(1)))/k_loc(2);...
     sin(k_loc(3))*cos(k_loc(3))*(cos(k_loc(2)*k_loc(1))-1),    cos(k_loc(3))^2*(1-cos(k_loc(2)*k_loc(1)))+cos(k_loc(2)*k_loc(1)),  sin(k_loc(3))*sin(k_loc(2)*k_loc(1)),   sin(k_loc(3))*(1-cos(k_loc(2)*k_loc(1)))/k_loc(2);...
     -cos(k_loc(3))*sin(k_loc(2)*k_loc(1)),                     -sin(k_loc(3))*sin(k_loc(2)*k_loc(1)),                              cos(k_loc(2)*k_loc(1)),                 sin(k_loc(2)*k_loc(1))/k_loc(2);...
     0,                                                         0,                                                                  0,                                      1];
 
H_loc_equ(k_loc) = ...
    [1, 0, 0, 0;...
     0, 1, 0, 0;...
     0, 0, 1, k_loc(1);...
     0, 0, 0, 1];
 
H_upper = [1, 0, 0, 0;...
           0, 1, 0, 0;...
           0, 0, 1, 0.051;...
           0, 0, 0, 1];
     
H_lower = [1, 0, 0, 0;...
           0, 1, 0, 0;...
           0, 0, 1, 0.041;...
           0, 0, 0, 1];
      
H_base = [1, 0, 0,  0;...
          0, 1, 0,  0;...
          0, 0, -1, 0;...
          0, 0, 0,  1];    
      
r = sym('r', [4 sections]);      
H = sym('H', [4 4 (sections+1)]);
H(:,:, 1) = H_base;

k_loc_ = f_loc(q(1), q(2), q(3));
%k_tilde = [k_loc_(1); k_loc_(2).^2 ./ sqrt(1 + k_loc_(2).^2); k_loc_(3)]; 

%H_loc1 = H_loc(k_tilde(1), k_tilde(2), k_tilde(3));
H_loc_ = H_loc(k_loc_(1), k_loc_(2), k_loc_(3));

df_loc_dq1 = diff(k_loc_, q(1));
df_loc_dq2 = diff(k_loc_, q(2));
df_loc_dq3 = diff(k_loc_, q(3));

ddf_loc_dq1dq1 = diff(df_loc_dq1, q(1));
ddf_loc_dq2dq1 = diff(df_loc_dq2, q(1));
ddf_loc_dq3dq1 = diff(df_loc_dq3, q(1));
ddf_loc_dq1dq2 = diff(df_loc_dq1, q(2));
ddf_loc_dq2dq2 = diff(df_loc_dq2, q(2));
ddf_loc_dq3dq2 = diff(df_loc_dq3, q(2));
ddf_loc_dq1dq3 = diff(df_loc_dq1, q(3));
ddf_loc_dq2dq3 = diff(df_loc_dq2, q(3));
ddf_loc_dq3dq3 = diff(df_loc_dq3, q(3));

dH_dq1 = sym('dH_dq1', [4 4]);
dH_dq2 = sym('dH_dq2', [4 4]);
dH_dq3 = sym('dH_dq3', [4 4]);
ddH_dq1dq1 = sym('dH_dq1dq1', [4 4]);
ddH_dq2dq1 = sym('dH_dq2dq1', [4 4]);
ddH_dq3dq1 = sym('dH_dq3dq1', [4 4]);
ddH_dq1dq2 = sym('dH_dq1dq2', [4 4]);
ddH_dq2dq2 = sym('dH_dq2dq2', [4 4]);
ddH_dq3dq2 = sym('dH_dq3dq2', [4 4]);
ddH_dq1dq3 = sym('dH_dq1dq3', [4 4]);
ddH_dq2dq3 = sym('dH_dq2dq3', [4 4]);
ddH_dq3dq3 = sym('dH_dq3dq3', [4 4]);


for i = 1:4
    for j = 1:4
        H_lock_loc = H_loc(k_loc(1), k_loc(2), k_loc(3));
        dH_loc_dk = jacobian(H_lock_loc(i, j), k_loc);
        ddH_loc_ddk = jacobian(dH_loc_dk, k_loc);

        dH_dq1(i, j) = dH_loc_dk * df_loc_dq1;
        dH_dq2(i, j) = dH_loc_dk * df_loc_dq2;
        dH_dq3(i, j) = dH_loc_dk * df_loc_dq3;

        ddH_dq1dq1(i, j) = (ddH_loc_ddk * df_loc_dq1)' * df_loc_dq1 + dH_loc_dk * ddf_loc_dq1dq1;
        ddH_dq2dq1(i, j) = (ddH_loc_ddk * df_loc_dq2)' * df_loc_dq1 + dH_loc_dk * ddf_loc_dq2dq1;
        ddH_dq3dq1(i, j) = (ddH_loc_ddk * df_loc_dq3)' * df_loc_dq1 + dH_loc_dk * ddf_loc_dq3dq1;
        ddH_dq1dq2(i, j) = (ddH_loc_ddk * df_loc_dq1)' * df_loc_dq2 + dH_loc_dk * ddf_loc_dq1dq2;
        ddH_dq2dq2(i, j) = (ddH_loc_ddk * df_loc_dq2)' * df_loc_dq2 + dH_loc_dk * ddf_loc_dq2dq2;
        ddH_dq3dq2(i, j) = (ddH_loc_ddk * df_loc_dq3)' * df_loc_dq2 + dH_loc_dk * ddf_loc_dq3dq2;
        ddH_dq1dq3(i, j) = (ddH_loc_ddk * df_loc_dq1)' * df_loc_dq3 + dH_loc_dk * ddf_loc_dq1dq3;
        ddH_dq2dq3(i, j) = (ddH_loc_ddk * df_loc_dq2)' * df_loc_dq3 + dH_loc_dk * ddf_loc_dq2dq3;
        ddH_dq3dq3(i, j) = (ddH_loc_ddk * df_loc_dq3)' * df_loc_dq3 + dH_loc_dk * ddf_loc_dq3dq3;
    end
end

dHloc_dq1 = subs(dH_dq1, k_loc, f_loc(q(1), q(2), q(3)));
dHloc_dq2 = subs(dH_dq2, k_loc, f_loc(q(1), q(2), q(3)));
dHloc_dq3 = subs(dH_dq3, k_loc, f_loc(q(1), q(2), q(3)));

ddHloc_dq1dq1 = subs(ddH_dq1dq1, k_loc, f_loc(q(1), q(2), q(3)));
ddHloc_dq2dq1 = subs(ddH_dq2dq1, k_loc, f_loc(q(1), q(2), q(3)));
ddHloc_dq3dq1 = subs(ddH_dq3dq1, k_loc, f_loc(q(1), q(2), q(3)));
ddHloc_dq1dq2 = subs(ddH_dq1dq2, k_loc, f_loc(q(1), q(2), q(3)));
ddHloc_dq2dq2 = subs(ddH_dq2dq2, k_loc, f_loc(q(1), q(2), q(3)));
ddHloc_dq3dq2 = subs(ddH_dq3dq2, k_loc, f_loc(q(1), q(2), q(3)));
ddHloc_dq1dq3 = subs(ddH_dq1dq3, k_loc, f_loc(q(1), q(2), q(3)));
ddHloc_dq2dq3 = subs(ddH_dq2dq3, k_loc, f_loc(q(1), q(2), q(3)));
ddHloc_dq3dq3 = subs(ddH_dq3dq3, k_loc, f_loc(q(1), q(2), q(3)));

ttt = matlabFunction(H_loc_,'File','kin/H','Vars',{q});
ttt = matlabFunction(dHloc_dq1,'File','kin/dH_dq1','Vars',{q});
ttt = matlabFunction(dHloc_dq2,'File','kin/dH_dq2','Vars',{q});
ttt = matlabFunction(dHloc_dq3,'File','kin/dH_dq3','Vars',{q});

ttt = matlabFunction(ddHloc_dq1dq1,'File','kin/ddH_dq1dq1','Vars',{q});
ttt = matlabFunction(ddHloc_dq2dq1,'File','kin/ddH_dq2dq1','Vars',{q});
ttt = matlabFunction(ddHloc_dq3dq1,'File','kin/ddH_dq3dq1','Vars',{q});
ttt = matlabFunction(ddHloc_dq1dq2,'File','kin/ddH_dq1dq2','Vars',{q});
ttt = matlabFunction(ddHloc_dq2dq2,'File','kin/ddH_dq2dq2','Vars',{q});
ttt = matlabFunction(ddHloc_dq3dq2,'File','kin/ddH_dq3dq2','Vars',{q});
ttt = matlabFunction(ddHloc_dq1dq3,'File','kin/ddH_dq1dq3','Vars',{q});
ttt = matlabFunction(ddHloc_dq2dq3,'File','kin/ddH_dq2dq3','Vars',{q});
ttt = matlabFunction(ddHloc_dq3dq3,'File','kin/ddH_dq3dq3','Vars',{q});

