function Q = Qfunc(in1,in2)
%QFUNC
%    Q = QFUNC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    13-Jun-2016 17:14:32

Fm1 = in2(1,:);
Fm2 = in2(2,:);
Fm3 = in2(3,:);
dq1 = in1(1,:);
dq2 = in1(2,:);
dq3 = in1(3,:);
Q = [Fm1-dq1.*1.0e3;Fm2-dq2.*1.0e3;Fm3-dq3.*1.0e3];
