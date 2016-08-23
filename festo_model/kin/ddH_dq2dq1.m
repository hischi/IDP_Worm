function ddHloc_dq2dq1 = ddH_dq2dq1(in1)
%DDH_DQ2DQ1
%    DDHLOC_DQ2DQ1 = DDH_DQ2DQ1(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    23-Aug-2016 18:41:27

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
t47 = q2.*2.0;
t35 = q1+q3-t47;
t36 = t18.*t34.*t35;
t37 = t33+t36;
t38 = t5.^2;
t39 = t38.*3.0;
t40 = t25+t39;
t41 = 1.0./t40;
t42 = 1.0./t3;
t43 = t4.*t42;
t44 = 1.0./t3.^2;
t49 = t4.*t5.*t44;
t45 = t43-t49;
t46 = t32-1.0;
t48 = 1.0./sqrt(t22);
t50 = q2.*4.0;
t51 = q3.*4.0;
t57 = q1.*8.0;
t52 = t50+t51-t57;
t53 = t23.*t30.*(1.0./2.0e1);
t54 = t3.*t18.*t48;
t55 = t53+t54;
t56 = t13.^2;
t58 = t4.*t44.*2.0;
t59 = 1.0./t3.^3;
t77 = t4.*t5.*t59.*4.0;
t60 = t58-t77;
t61 = t25.*t41.*t60;
t62 = 1.0./t40.^2;
t63 = t25.*t45.*t52.*t62;
t76 = t41.*t45.*t52;
t64 = t61+t63-t76;
t65 = t9.*t13.*t25.*t31;
t86 = t9.*t13.*t31.*t38.*3.0;
t66 = t65-t86;
t67 = 1.0./t17.^3;
t68 = t23.*t67.*(1.0./4.0e2);
t69 = t30.*t35.*t48.*(1.0./4.0e1);
t70 = t3.*t30.*t48.*(1.0./4.0e1);
t71 = 1.0./t22.^(3.0./2.0);
t74 = t18.*t48;
t75 = t3.*t18.*t35.*t71.*(1.0./2.0);
t72 = t68+t69+t70-t74-t75;
t73 = 1.0./t8;
t78 = t17.^2;
t79 = 1.0./t22;
t80 = t4.*t5.*t46.*t73.*t78.*t79.*(1.0./4.0);
t81 = t4.*t5.*t13.*t17.*t31.*t48.*t73.*(1.0./2.0);
t82 = t80+t81;
t83 = t3.*t4.*t5.*t9.*t31;
t84 = t3.*t4.*t5.*t9.*t13.*t18.*t23.*t32.*2.0;
t85 = t83+t84;
t87 = t25.*t41.*t45.*t66;
t88 = t3.*t4.*t5.*t9.*t31.*(1.0./3.0);
t89 = t3.*t4.*t5.*t9.*t13.*t18.*t23.*t32.*(2.0./3.0);
t90 = t87+t88+t89-t3.*t4.*t5.*t9.*t32.*t37.*t56;
t91 = t9.*t25.*t46;
t92 = t91-t9.*t38.*t46.*3.0;
t93 = t9.*t18.*t23.*t25.*t31.*2.0;
t94 = t25.*t41.*t45.*(t93-t9.*t18.*t23.*t31.*t38.*6.0).*(1.0./3.0);
t95 = t37.*t66;
t96 = t9.*t18.*t23.*t31.*t38.*2.0;
t97 = t95+t96-t9.*t18.*t23.*t25.*t31.*(2.0./3.0)-t3.*t4.*t5.*t9.*t25.*t41.*t45.*t46.*4.0;
t98 = t3.*t4.*t5.*t9.*t13.*t31.*t72;
t99 = t3.*t4.*t5.*t9.*t30.*t32.*(t19+t20+t21-t26-t27-t28).*(4.0./9.0);
t100 = t94+t98+t99-t37.*t85.*(1.0./3.0)-t55.*t90-t64.*t92-t4.*t5.*t41.*t97.*2.0;
t101 = t9.*t25.*t31.*(1.0./3.0);
t102 = t9.*t13.*t18.*t23.*t25.*t32.*(2.0./3.0);
t103 = t9.*t25.*t31;
t104 = t9.*t13.*t18.*t23.*t25.*t32.*2.0;
t105 = t9.*t25.*t46.*2.0;
t106 = t105-t9.*t38.*t46.*6.0;
t107 = t25.*t41.*t45.*t106;
t108 = t3.*t4.*t5.*t9.*t13.*t31.*t37.*2.0;
t109 = t3.*t4.*t5.*t9.*t18.*t23.*t25.*t31.*t41.*t45.*(4.0./3.0);
t110 = t3.*t46.*t73.*t78.*t79.*(1.0./4.0);
t111 = t3.*t13.*t17.*t31.*t48.*t73.*(1.0./2.0);
t112 = t110+t111;
t113 = t3.*t32.*t73;
t114 = t113-t3.*t13.*t18.*t23.*t31.*t73.*2.0;
t115 = t3.*t32.*t73.*(1.0./3.0);
t116 = t3.*t31.*t37.*t56.*t73;
t117 = t115+t116-t3.*t13.*t18.*t23.*t31.*t73.*(2.0./3.0)-t4.*t5.*t13.*t25.*t32.*t41.*t45.*t73;
t118 = t3.*t25.*t31.*t41.*t45.*t73;
t119 = t4.*t5.*t18.*t23.*t32.*t73.*(2.0./3.0);
t120 = t118+t119-t4.*t5.*t13.*t32.*t37.*t73;
t121 = t3.*t13.*t32.*t72.*t73;
t122 = t4.*t5.*t32.*t73.*(1.0./3.0);
t123 = t3.*t13.*t25.*t32.*t41.*t45.*t73;
t124 = t4.*t5.*t31.*t37.*t56.*t73;
t125 = t122+t123+t124-t4.*t5.*t13.*t18.*t23.*t31.*t73.*(2.0./3.0);
t126 = t55.*t125;
t127 = t4.*t5.*t32.*t73;
t128 = t127-t4.*t5.*t13.*t18.*t23.*t31.*t73.*2.0;
t129 = t37.*t128.*(1.0./3.0);
t130 = t3.*t13.*t32.*t37.*t73;
t131 = t4.*t5.*t25.*t31.*t41.*t45.*t73;
t132 = t130+t131-t3.*t18.*t23.*t32.*t73.*(2.0./3.0);
t133 = t4.*t5.*t41.*t132.*2.0;
t134 = t13.*t18.*t23.*t32.*2.0;
t135 = t31.*(1.0./3.0);
ddHloc_dq2dq1 = reshape([t109+t37.*(t103+t104).*(1.0./3.0)+t55.*(t101+t102-t9.*t25.*t32.*t37.*t56-t3.*t4.*t5.*t9.*t13.*t25.*t31.*t41.*t45.*2.0)-t4.*t5.*t41.*(t107+t108-t3.*t4.*t5.*t9.*t18.*t23.*t31.*(4.0./3.0)).*2.0-t9.*t22.*t25.*t30.*t32.*(4.0./9.0)-t9.*t13.*t25.*t31.*t72-t3.*t4.*t5.*t9.*t46.*t64.*2.0,t100,-t121+t37.*t114.*(1.0./3.0)+t55.*t117+t4.*t5.*t41.*t120.*2.0+t3.*t30.*t31.*t73.*(t19+t20+t21-t26-t27-t28).*(4.0./9.0)+t4.*t5.*t31.*t64.*t73+t4.*t5.*t18.*t23.*t25.*t32.*t41.*t45.*t73.*(2.0./3.0),0.0,t100,-t109-t55.*(t101+t102-t135+t37.*(t32.*t56-t9.*t25.*t32.*t56)-t13.*t18.*t23.*t32.*(2.0./3.0)-t3.*t4.*t5.*t9.*t13.*t25.*t31.*t41.*t45.*2.0)+t37.*(t31-t103-t104+t134).*(1.0./3.0)+t72.*(t65-t13.*t31)-t22.*t30.*t32.*(4.0./9.0)+t4.*t5.*t41.*(t107+t108-t3.*t4.*t5.*t9.*t18.*t23.*t31.*(4.0./3.0)).*2.0+t9.*t25.*t30.*t32.*(t19+t20+t21-t26-t27-t28).*(4.0./9.0)+t3.*t4.*t5.*t9.*t46.*t64.*2.0,-t126-t129-t133+t3.*t31.*t64.*t73-t4.*t5.*t22.*t30.*t31.*t73.*(4.0./9.0)+t4.*t5.*t13.*t32.*t72.*t73+t3.*t18.*t23.*t25.*t32.*t41.*t45.*t73.*(2.0./3.0),0.0,t121-t37.*t114.*(1.0./3.0)-t55.*t117-t4.*t5.*t41.*t120.*2.0-t3.*t22.*t30.*t31.*t73.*(4.0./9.0)-t4.*t5.*t31.*t64.*t73-t4.*t5.*t18.*t23.*t25.*t32.*t41.*t45.*t73.*(2.0./3.0),t126+t129+t133-t3.*t31.*t64.*t73-t4.*t5.*t13.*t32.*t72.*t73+t4.*t5.*t30.*t31.*t73.*(t19+t20+t21-t26-t27-t28).*(4.0./9.0)-t3.*t18.*t23.*t25.*t32.*t41.*t45.*t73.*(2.0./3.0),t37.*(t31+t134).*(1.0./3.0)+t55.*(t135-t32.*t37.*t56+t13.*t18.*t23.*t32.*(2.0./3.0))-t22.*t30.*t32.*(4.0./9.0)-t13.*t31.*t72,0.0,-t55.*(t37.*(t3.*t17.*t32.*t48.*t56.*t73.*(-1.0./2.0)+t3.*t13.*t31.*t73.*t78.*t79.*(1.0./2.0)+t3.*t17.*t46.*t71.*t73.*t78.*(1.0./4.0))+t3.*t13.*t32.*t73.*(1.0./3.0)-t25.*t41.*t45.*t82)+t72.*t112+t4.*t5.*t41.*(t37.*t82-t4.*t5.*t31.*t73.*(1.0./3.0)+t3.*t17.*t25.*t41.*t45.*t46.*t48.*t73.*(1.0./2.0)).*2.0+t3.*t18.*t23.*t32.*t73.*(2.0./9.0)-t3.*t13.*t32.*t37.*t73.*(1.0./3.0)-t4.*t5.*t25.*t31.*t41.*t45.*t73.*(1.0./3.0)+t4.*t5.*t17.*t46.*t48.*t64.*t73.*(1.0./2.0),-t72.*t82+t55.*(t37.*(t4.*t5.*t17.*t32.*t48.*t56.*t73.*(-1.0./2.0)+t4.*t5.*t13.*t31.*t73.*t78.*t79.*(1.0./2.0)+t4.*t5.*t17.*t46.*t71.*t73.*t78.*(1.0./4.0))+t25.*t41.*t45.*t112+t4.*t5.*t13.*t32.*t73.*(1.0./3.0))-t4.*t5.*t41.*(-t37.*t112+t3.*t31.*t73.*(1.0./3.0)+t4.*t5.*t17.*t25.*t41.*t45.*t46.*t48.*t73.*(1.0./2.0)).*2.0-t4.*t5.*t18.*t23.*t32.*t73.*(2.0./9.0)+t4.*t5.*t13.*t32.*t37.*t73.*(1.0./3.0)-t3.*t25.*t31.*t41.*t45.*t73.*(1.0./3.0)+t3.*t17.*t46.*t48.*t64.*t73.*(1.0./2.0),-t72.*(t31.*t78.*t79.*(1.0./4.0)-t13.*t17.*t32.*t48.*(1.0./2.0))+t55.*(t13.*t31.*(1.0./3.0)-t37.*(t17.*t31.*t48.*t56.*(1.0./2.0)-t17.*t31.*t71.*t78.*(1.0./4.0)+t13.*t32.*t78.*t79.*(1.0./2.0)))-t18.*t23.*t31.*(2.0./9.0)+t13.*t31.*t37.*(1.0./3.0),0.0],[4,4]);