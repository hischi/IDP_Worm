function ddHloc_dq1dq1 = ddH_dq1dq1(in1)
%DDH_DQ1DQ1
%    DDHLOC_DQ1DQ1 = DDH_DQ1DQ1(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    23-Aug-2016 18:40:50

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1.*(1.0./4.0e1);
t3 = q2.*(1.0./4.0e1);
t4 = q3.*(1.0./4.0e1);
t5 = t2+t3+t4;
t6 = q1.^2;
t7 = q2.^2;
t8 = q3.^2;
t11 = q1.*q2;
t12 = q1.*q3;
t13 = q2.*q3;
t9 = t6+t7+t8-t11-t12-t13;
t10 = 1.0./t5;
t14 = sqrt(t9);
t15 = q1.*2.0;
t16 = q2+q3-t15;
t17 = sqrt(3.0);
t18 = q2-q3;
t19 = t17.*t18.*1i;
t20 = -q2-q3+t15+t19;
t21 = abs(t20);
t22 = 1.0./t21.^2;
t23 = q1.*(1.0./3.0);
t24 = q2.*(1.0./3.0);
t25 = q3.*(1.0./3.0);
t26 = t23+t24+t25;
t27 = t10.*t14.*t26.*2.0;
t28 = 1.0./t5.^2;
t29 = conj(t14);
t30 = t16.^2;
t31 = cos(t27);
t32 = sin(t27);
t33 = t18.^2;
t34 = t28.*t29.*(1.0./2.0e1);
t35 = 1.0./t29;
t36 = t10.*t16.*t35;
t37 = t34+t36;
t38 = t33.*3.0;
t39 = t30+t38;
t40 = 1.0./t39;
t41 = t31-1.0;
t42 = 1.0./sqrt(t9);
t43 = t14.*t28.*(1.0./2.0e1);
t44 = t26.^2;
t45 = t22.*t26.*t30.*t32;
t70 = t22.*t26.*t32.*t33.*3.0;
t46 = t45-t70;
t47 = 1.0./t39.^2;
t48 = q2.*4.0;
t49 = q3.*4.0;
t60 = q1.*8.0;
t50 = t48+t49-t60;
t51 = t10.*t42.*2.0;
t52 = 1.0./t5.^3;
t53 = t14.*t52.*(1.0./4.0e2);
t54 = t16.*t28.*t42.*(1.0./2.0e1);
t55 = 1.0./t9.^(3.0./2.0);
t61 = t10.*t30.*t55.*(1.0./2.0);
t56 = t51+t53+t54-t61;
t57 = 1.0./t21;
t58 = t10.*t16.*t42;
t59 = t43+t58;
t62 = t5.^2;
t63 = 1.0./t9;
t64 = t17.*t18.*t41.*t57.*t62.*t63.*(1.0./4.0);
t65 = t5.*t17.*t18.*t26.*t32.*t42.*t57.*(1.0./2.0);
t66 = t64+t65;
t67 = t16.*t17.*t18.*t22.*t32;
t68 = t10.*t14.*t16.*t17.*t18.*t22.*t26.*t31.*2.0;
t69 = t67+t68;
t71 = t17.*t18.*t40.*t46.*2.0;
t72 = t16.*t17.*t18.*t22.*t32.*(1.0./3.0);
t73 = t10.*t14.*t16.*t17.*t18.*t22.*t26.*t31.*(2.0./3.0);
t74 = t71+t72+t73-t16.*t17.*t18.*t22.*t31.*t37.*t44;
t75 = t10.*t14.*t22.*t30.*t32.*2.0;
t76 = t17.*t18.*t40.*(t75-t10.*t14.*t22.*t32.*t33.*6.0).*(2.0./3.0);
t77 = t37.*t46;
t78 = t10.*t14.*t22.*t32.*t33.*2.0;
t79 = t77+t78-t10.*t14.*t22.*t30.*t32.*(2.0./3.0)-t16.*t22.*t33.*t40.*t41.*2.4e1;
t80 = t22.*t30.*t41;
t81 = t80-t22.*t33.*t41.*3.0;
t82 = t16.*t17.*t18.*t22.*t26.*t32.*t56;
t83 = t16.*t17.*t18.*t22.*t28.*t31.*(t6+t7+t8-t11-t12-t13).*(4.0./9.0);
t84 = t76+t82+t83-t37.*t69.*(1.0./3.0)-t59.*t74-t17.*t18.*t40.*t79.*2.0-t17.*t18.*t47.*t50.*t81.*2.0;
t85 = t22.*t30.*t32;
t86 = t10.*t14.*t22.*t26.*t30.*t31.*2.0;
t87 = t22.*t30.*t32.*(1.0./3.0);
t88 = t10.*t14.*t22.*t26.*t30.*t31.*(2.0./3.0);
t89 = t22.*t30.*t41.*2.0;
t90 = t89-t22.*t33.*t41.*6.0;
t91 = t17.*t18.*t40.*t90.*2.0;
t92 = t16.*t17.*t18.*t22.*t26.*t32.*t37.*2.0;
t93 = t10.*t14.*t16.*t22.*t32.*t33.*t40.*8.0;
t94 = t16.*t41.*t57.*t62.*t63.*(1.0./4.0);
t95 = t5.*t16.*t26.*t32.*t42.*t57.*(1.0./2.0);
t96 = t94+t95;
t97 = t16.*t31.*t57;
t98 = t97-t10.*t14.*t16.*t26.*t32.*t57.*2.0;
t99 = t16.*t31.*t57.*(1.0./3.0);
t100 = t16.*t32.*t37.*t44.*t57;
t101 = t99+t100-t26.*t31.*t33.*t40.*t57.*6.0-t10.*t14.*t16.*t26.*t32.*t57.*(2.0./3.0);
t102 = t10.*t14.*t17.*t18.*t31.*t57.*(2.0./3.0);
t103 = t16.*t17.*t18.*t32.*t40.*t57.*2.0;
t104 = t102+t103-t17.*t18.*t26.*t31.*t37.*t57;
t105 = t16.*t26.*t31.*t56.*t57;
t106 = t17.*t18.*t31.*t57;
t107 = t106-t10.*t14.*t17.*t18.*t26.*t32.*t57.*2.0;
t108 = t37.*t107.*(1.0./3.0);
t109 = t17.*t18.*t31.*t57.*(1.0./3.0);
t110 = t17.*t18.*t32.*t37.*t44.*t57;
t111 = t16.*t17.*t18.*t26.*t31.*t40.*t57.*2.0;
t112 = t109+t110+t111-t10.*t14.*t17.*t18.*t26.*t32.*t57.*(2.0./3.0);
t113 = t59.*t112;
t114 = t32.*t33.*t40.*t57.*6.0;
t115 = t16.*t26.*t31.*t37.*t57;
t116 = t114+t115-t10.*t14.*t16.*t31.*t57.*(2.0./3.0);
t117 = t17.*t18.*t40.*t116.*2.0;
t118 = t10.*t14.*t26.*t31.*2.0;
t119 = t32.*(1.0./3.0);
ddHloc_dq1dq1 = reshape([t93+t37.*(t85+t86).*(1.0./3.0)+(t43+t10.*t42.*(q1.*-2.0+q2+q3)).*(t87+t88-t22.*t30.*t31.*t37.*t44-t16.*t22.*t26.*t32.*t33.*t40.*1.2e1)-t17.*t18.*t40.*(t91+t92-t10.*t14.*t16.*t17.*t18.*t22.*t32.*(4.0./3.0)).*2.0-t9.*t22.*t28.*t30.*t31.*(4.0./9.0)-t22.*t26.*t30.*t32.*t56-t16.*t22.*t33.*t41.*t47.*t50.*1.2e1,t84,-t105+t37.*t98.*(1.0./3.0)+t59.*t101+t17.*t18.*t40.*t104.*2.0+t16.*t28.*t32.*t57.*(t6+t7+t8-t11-t12-t13).*(4.0./9.0)+t32.*t33.*t47.*t50.*t57.*6.0+t10.*t14.*t31.*t33.*t40.*t57.*4.0,0.0,t84,-t93+t37.*(t32-t85-t86+t118).*(1.0./3.0)-t59.*(t87+t88-t119+t37.*(t31.*t44-t22.*t30.*t31.*t44)-t10.*t14.*t26.*t31.*(2.0./3.0)-t16.*t22.*t26.*t32.*t33.*t40.*1.2e1)+t56.*(t45-t26.*t32)-t9.*t28.*t31.*(4.0./9.0)+t17.*t18.*t40.*(t91+t92-t10.*t14.*t16.*t17.*t18.*t22.*t32.*(4.0./3.0)).*2.0+t22.*t28.*t30.*t31.*(t6+t7+t8-t11-t12-t13).*(4.0./9.0)+t16.*t22.*t33.*t41.*t47.*t50.*1.2e1,-t108-t113-t117-t9.*t17.*t18.*t28.*t32.*t57.*(4.0./9.0)+t17.*t18.*t26.*t31.*t56.*t57+t16.*t17.*t18.*t32.*t47.*t50.*t57.*2.0+t10.*t14.*t16.*t17.*t18.*t31.*t40.*t57.*(4.0./3.0),0.0,t105-t37.*t98.*(1.0./3.0)-t59.*t101-t17.*t18.*t40.*t104.*2.0-t9.*t16.*t28.*t32.*t57.*(4.0./9.0)-t32.*t33.*t47.*t50.*t57.*6.0-t10.*t14.*t31.*t33.*t40.*t57.*4.0,t108+t113+t117-t17.*t18.*t26.*t31.*t56.*t57+t17.*t18.*t28.*t32.*t57.*(t6+t7+t8-t11-t12-t13).*(4.0./9.0)-t16.*t17.*t18.*t32.*t47.*t50.*t57.*2.0-t10.*t14.*t16.*t17.*t18.*t31.*t40.*t57.*(4.0./3.0),t37.*(t32+t118).*(1.0./3.0)+t59.*(t119-t31.*t37.*t44+t10.*t14.*t26.*t31.*(2.0./3.0))-t9.*t28.*t31.*(4.0./9.0)-t26.*t32.*t56,0.0,-t59.*(t37.*(t5.*t16.*t31.*t42.*t44.*t57.*(-1.0./2.0)+t5.*t16.*t41.*t55.*t57.*t62.*(1.0./4.0)+t16.*t26.*t32.*t57.*t62.*t63.*(1.0./2.0))+t16.*t26.*t31.*t57.*(1.0./3.0)-t17.*t18.*t40.*t66.*2.0)+t56.*t96-t32.*t33.*t40.*t57.*2.0+t17.*t18.*t40.*(t37.*t66-t17.*t18.*t32.*t57.*(1.0./3.0)+t5.*t16.*t17.*t18.*t40.*t41.*t42.*t57).*2.0+t10.*t14.*t16.*t31.*t57.*(2.0./9.0)-t16.*t26.*t31.*t37.*t57.*(1.0./3.0)+t5.*t33.*t41.*t42.*t47.*t50.*t57.*3.0,-t56.*t66+t59.*(t37.*(t5.*t17.*t18.*t31.*t42.*t44.*t57.*(-1.0./2.0)+t5.*t17.*t18.*t41.*t55.*t57.*t62.*(1.0./4.0)+t17.*t18.*t26.*t32.*t57.*t62.*t63.*(1.0./2.0))+t17.*t18.*t40.*t96.*2.0+t17.*t18.*t26.*t31.*t57.*(1.0./3.0))-t17.*t18.*t40.*(-t37.*t96+t16.*t32.*t57.*(1.0./3.0)+t5.*t33.*t40.*t41.*t42.*t57.*3.0).*2.0-t10.*t14.*t17.*t18.*t31.*t57.*(2.0./9.0)-t16.*t17.*t18.*t32.*t40.*t57.*(2.0./3.0)+t17.*t18.*t26.*t31.*t37.*t57.*(1.0./3.0)+t5.*t16.*t17.*t18.*t41.*t42.*t47.*t50.*t57,-t56.*(t32.*t62.*t63.*(1.0./4.0)-t5.*t26.*t31.*t42.*(1.0./2.0))+t59.*(t26.*t32.*(1.0./3.0)-t37.*(t5.*t32.*t42.*t44.*(1.0./2.0)-t5.*t32.*t55.*t62.*(1.0./4.0)+t26.*t31.*t62.*t63.*(1.0./2.0)))-t10.*t14.*t32.*(2.0./9.0)+t26.*t32.*t37.*(1.0./3.0),0.0],[4,4]);