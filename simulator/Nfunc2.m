function N = Nfunc2(in1,in2)
%NFUNC2
%    N = NFUNC2(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    13-Jun-2016 17:14:10

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
N = [q1.*1.5e3-2.404905e2;q2.*1.5e3-2.404905e2;q3.*1.5e3-2.404905e2];
