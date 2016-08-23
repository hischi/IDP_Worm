function ddHloc_dq3dq2 = ddH_dq3dq2(in1)
%DDH_DQ3DQ2
%    DDHLOC_DQ3DQ2 = DDH_DQ3DQ2(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    23-Aug-2016 18:44:14

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1.*2.0;
t3 = q2+q3-t2;
t4 = sqrt(3.0);
t5 = q2-q3;
t6 = t4.*t5.*1i;
t7 = -q2-q3+t2+t6;
t8 = abs(t7);
t9 = 1.0./t8.^2;
t10 = q1.*(1.0./3.0);
t11 = q2.*(1.0./3.0);
t12 = q3.*(1.0./3.0);
t13 = t10+t11+t12;
t14 = q1.*(1.0./4.0e1);
t15 = q2.*(1.0./4.0e1);
t16 = q3.*(1.0./4.0e1);
t17 = t14+t15+t16;
t18 = 1.0./t17;
t19 = q1.^2;
t20 = q2.^2;
t21 = q3.^2;
t26 = q1.*q2;
t27 = q1.*q3;
t28 = q2.*q3;
t22 = t19+t20+t21-t26-t27-t28;
t23 = sqrt(t22);
t24 = t13.*t18.*t23.*2.0;
t25 = t3.^2;
t29 = conj(t23);
t30 = 1.0./t17.^2;
t31 = sin(t24);
t32 = cos(t24);
t33 = t29.*t30.*(1.0./2.0e1);
t34 = 1.0./t29;
t48 = q3.*2.0;
t35 = q1+q2-t48;
t36 = t18.*t34.*t35;
t37 = t33+t36;
t38 = t5.^2;
t39 = t38.*3.0;
t40 = t25+t39;
t41 = 1.0./t40;
t42 = 1.0./t3;
t43 = t4.*t42;
t44 = 1.0./t3.^2;
t45 = t4.*t5.*t44;
t46 = t43+t45;
t47 = t32-1.0;
t49 = 1.0./sqrt(t22);
t51 = q2.*2.0;
t50 = q1+q3-t51;
t52 = q1.*4.0;
t53 = 1.0./t40.^2;
t54 = q3.*4.0;
t75 = q2.*8.0;
t55 = t52+t54-t75;
t56 = t25.*t46.*t53.*t55;
t57 = t23.*t30.*(1.0./2.0e1);
t58 = t18.*t49.*t50;
t59 = t57+t58;
t60 = t13.^2;
t61 = t43-t45;
t62 = t9.*t13.*t25.*t31;
t88 = t9.*t13.*t31.*t38.*3.0;
t63 = t62-t88;
t64 = 1.0./t17.^3;
t65 = t23.*t64.*(1.0./4.0e2);
t66 = t30.*t35.*t49.*(1.0./4.0e1);
t67 = t30.*t49.*t50.*(1.0./4.0e1);
t68 = 1.0./t22.^(3.0./2.0);
t71 = t18.*t49;
t72 = t18.*t35.*t50.*t68.*(1.0./2.0);
t69 = t65+t66+t67-t71-t72;
t70 = 1.0./t8;
t73 = t48+t51-t52;
t74 = t41.*t46.*t73;
t82 = t4.*t5.*t41.*t42.*2.0;
t76 = t56+t74-t82;
t77 = t17.^2;
t78 = 1.0./t22;
t79 = t4.*t5.*t47.*t70.*t77.*t78.*(1.0./4.0);
t80 = t4.*t5.*t13.*t17.*t31.*t49.*t70.*(1.0./2.0);
t81 = t79+t80;
t83 = t9.*t25.*t47;
t84 = t76.*(t83-t9.*t38.*t47.*3.0);
t85 = t3.*t4.*t5.*t9.*t31;
t86 = t3.*t4.*t5.*t9.*t13.*t18.*t23.*t32.*2.0;
t87 = t85+t86;
t89 = t25.*t41.*t46.*t63;
t90 = t3.*t4.*t5.*t9.*t32.*t37.*t60;
t91 = t89+t90-t3.*t4.*t5.*t9.*t31.*(1.0./3.0)-t3.*t4.*t5.*t9.*t13.*t18.*t23.*t32.*(2.0./3.0);
t92 = t59.*t91;
t93 = t9.*t18.*t23.*t25.*t31.*2.0;
t94 = t93-t9.*t18.*t23.*t31.*t38.*6.0;
t95 = t37.*t63;
t96 = t9.*t18.*t23.*t31.*t38.*2.0;
t97 = t3.*t4.*t5.*t9.*t25.*t41.*t46.*t47.*4.0;
t98 = t95+t96+t97-t9.*t18.*t23.*t25.*t31.*(2.0./3.0);
t99 = t3.*t4.*t5.*t9.*t13.*t31.*t69;
t100 = t3.*t4.*t5.*t9.*t30.*t32.*(t19+t20+t21-t26-t27-t28).*(4.0./9.0);
t101 = t84+t92+t99+t100-t37.*t87.*(1.0./3.0)-t25.*t41.*t46.*t94.*(1.0./3.0)-t25.*t41.*t61.*t98;
t102 = t9.*t25.*t31.*(1.0./3.0);
t103 = t9.*t13.*t18.*t23.*t25.*t32.*(2.0./3.0);
t104 = t3.*t4.*t5.*t9.*t13.*t25.*t31.*t41.*t46.*2.0;
t105 = t9.*t25.*t31;
t106 = t9.*t13.*t18.*t23.*t25.*t32.*2.0;
t107 = t9.*t25.*t47.*2.0;
t108 = t107-t9.*t38.*t47.*6.0;
t109 = t25.*t41.*t46.*t108;
t110 = t3.*t4.*t5.*t9.*t18.*t23.*t31.*(4.0./3.0);
t111 = t109+t110-t3.*t4.*t5.*t9.*t13.*t31.*t37.*2.0;
t112 = t25.*t41.*t61.*t111;
t113 = t3.*t47.*t70.*t77.*t78.*(1.0./4.0);
t114 = t3.*t13.*t17.*t31.*t49.*t70.*(1.0./2.0);
t115 = t113+t114;
t116 = t3.*t32.*t70;
t117 = t116-t3.*t13.*t18.*t23.*t31.*t70.*2.0;
t118 = t3.*t32.*t70.*(1.0./3.0);
t119 = t3.*t31.*t37.*t60.*t70;
t120 = t4.*t5.*t13.*t25.*t32.*t41.*t46.*t70;
t121 = t118+t119+t120-t3.*t13.*t18.*t23.*t31.*t70.*(2.0./3.0);
t122 = t3.*t25.*t31.*t41.*t46.*t70;
t123 = t4.*t5.*t13.*t32.*t37.*t70;
t124 = t122+t123-t4.*t5.*t18.*t23.*t32.*t70.*(2.0./3.0);
t125 = t25.*t41.*t61.*t124;
t126 = t3.*t13.*t32.*t69.*t70;
t127 = t4.*t5.*t31.*t70.*t76;
t128 = t4.*t5.*t18.*t23.*t25.*t32.*t41.*t46.*t70.*(2.0./3.0);
t129 = t4.*t5.*t32.*t70.*(1.0./3.0);
t130 = t4.*t5.*t31.*t37.*t60.*t70;
t131 = t129+t130-t4.*t5.*t13.*t18.*t23.*t31.*t70.*(2.0./3.0)-t3.*t13.*t25.*t32.*t41.*t46.*t70;
t132 = t59.*t131;
t133 = t4.*t5.*t32.*t70;
t134 = t133-t4.*t5.*t13.*t18.*t23.*t31.*t70.*2.0;
t135 = t37.*t134.*(1.0./3.0);
t136 = t3.*t18.*t23.*t32.*t70.*(2.0./3.0);
t137 = t4.*t5.*t25.*t31.*t41.*t46.*t70;
t138 = t136+t137-t3.*t13.*t32.*t37.*t70;
t139 = t3.*t31.*t70.*t76;
t140 = t3.*t18.*t23.*t25.*t32.*t41.*t46.*t70.*(2.0./3.0);
t141 = t13.*t18.*t23.*t32.*2.0;
ddHloc_dq3dq2 = reshape([t112+t37.*(t105+t106).*(1.0./3.0)+t59.*(t102+t103+t104-t9.*t25.*t32.*t37.*t60)-t9.*t22.*t25.*t30.*t32.*(4.0./9.0)-t9.*t13.*t25.*t31.*t69+t3.*t4.*t5.*t9.*t47.*(t56+t41.*t46.*(q1.*-4.0+t48+t51)-t4.*t5.*t41.*t42.*2.0).*2.0-t3.*t4.*t5.*t9.*t18.*t23.*t25.*t31.*t41.*t46.*(4.0./3.0),t101,-t125-t126-t127-t128+t37.*t117.*(1.0./3.0)+t59.*t121+t3.*t30.*t31.*t70.*(t19+t20+t21-t26-t27-t28).*(4.0./9.0),0.0,t101,-t112-t59.*(t31.*(-1.0./3.0)+t102+t103+t104+t37.*(t32.*t60-t9.*t25.*t32.*t60)-t13.*t18.*t23.*t32.*(2.0./3.0))+t37.*(t31-t105-t106+t141).*(1.0./3.0)+t69.*(t62-t13.*t31)-t22.*t30.*t32.*(4.0./9.0)+t9.*t25.*t30.*t32.*(t19+t20+t21-t26-t27-t28).*(4.0./9.0)-t3.*t4.*t5.*t9.*t47.*t76.*2.0+t3.*t4.*t5.*t9.*t18.*t23.*t25.*t31.*t41.*t46.*(4.0./3.0),-t132-t135-t139-t140+t25.*t41.*t61.*t138-t4.*t5.*t22.*t30.*t31.*t70.*(4.0./9.0)+t4.*t5.*t13.*t32.*t69.*t70,0.0,t125+t126+t127+t128-t37.*t117.*(1.0./3.0)-t59.*t121-t3.*t22.*t30.*t31.*t70.*(4.0./9.0),t132+t135+t139+t140-t25.*t41.*t61.*t138-t4.*t5.*t13.*t32.*t69.*t70+t4.*t5.*t30.*t31.*t70.*(t19+t20+t21-t26-t27-t28).*(4.0./9.0),t37.*(t31+t141).*(1.0./3.0)+t59.*(t31.*(1.0./3.0)-t32.*t37.*t60+t13.*t18.*t23.*t32.*(2.0./3.0))-t22.*t30.*t32.*(4.0./9.0)-t13.*t31.*t69,0.0,-t59.*(t37.*(t3.*t17.*t32.*t49.*t60.*t70.*(-1.0./2.0)+t3.*t13.*t31.*t70.*t77.*t78.*(1.0./2.0)+t3.*t17.*t47.*t68.*t70.*t77.*(1.0./4.0))+t3.*t13.*t32.*t70.*(1.0./3.0)+t25.*t41.*t46.*t81)+t69.*t115-t25.*t41.*t61.*(-t37.*t81+t4.*t5.*t31.*t70.*(1.0./3.0)+t3.*t17.*t25.*t41.*t46.*t47.*t49.*t70.*(1.0./2.0))+t3.*t18.*t23.*t32.*t70.*(2.0./9.0)-t3.*t13.*t32.*t37.*t70.*(1.0./3.0)+t4.*t5.*t25.*t31.*t41.*t46.*t70.*(1.0./3.0)-t4.*t5.*t17.*t47.*t49.*t70.*t76.*(1.0./2.0),-t69.*t81+t59.*(t37.*(t4.*t5.*t17.*t32.*t49.*t60.*t70.*(-1.0./2.0)+t4.*t5.*t13.*t31.*t70.*t77.*t78.*(1.0./2.0)+t4.*t5.*t17.*t47.*t68.*t70.*t77.*(1.0./4.0))-t25.*t41.*t46.*t115+t4.*t5.*t13.*t32.*t70.*(1.0./3.0))+t25.*t41.*t61.*(t37.*t115-t3.*t31.*t70.*(1.0./3.0)+t4.*t5.*t17.*t25.*t41.*t46.*t47.*t49.*t70.*(1.0./2.0))-t4.*t5.*t18.*t23.*t32.*t70.*(2.0./9.0)+t4.*t5.*t13.*t32.*t37.*t70.*(1.0./3.0)+t3.*t25.*t31.*t41.*t46.*t70.*(1.0./3.0)-t3.*t17.*t47.*t49.*t70.*t76.*(1.0./2.0),-t69.*(t31.*t77.*t78.*(1.0./4.0)-t13.*t17.*t32.*t49.*(1.0./2.0))+t59.*(t13.*t31.*(1.0./3.0)-t37.*(t17.*t31.*t49.*t60.*(1.0./2.0)-t17.*t31.*t68.*t77.*(1.0./4.0)+t13.*t32.*t77.*t78.*(1.0./2.0)))-t18.*t23.*t31.*(2.0./9.0)+t13.*t31.*t37.*(1.0./3.0),0.0],[4,4]);