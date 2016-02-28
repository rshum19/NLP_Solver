function Fs = Fs_fun(I1,I2,I3,a1,a3,da1,da3,l1,l2,lc1,lc2,lc3,m1,m2,m3)
%FS_FUN
%    FS = FS_FUN(I1,I2,I3,A1,A3,DA1,DA3,L1,L2,LC1,LC2,LC3,M1,M2,M3)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    26-Feb-2016 22:57:20

t2 = l1-lc1;
t3 = sin(a1);
t4 = cos(a1);
t5 = l1.^2;
t6 = t4.^2;
t7 = lc1.^2;
t8 = t3.^2;
t9 = cos(a3);
t10 = sin(a3);
t11 = l2.^2;
t12 = lc2.^2;
t13 = m1.*m2.*t5.*t6;
t14 = m1.*m3.*t5.*t6;
t15 = m1.*m2.*t6.*t7;
t16 = m1.*m3.*t6.*t7;
t17 = lc3.^2;
t18 = t9.^2;
t19 = m1.*m2.*t5.*t8;
t20 = m1.*m3.*t5.*t8;
t21 = m1.*m2.*t7.*t8;
t22 = m1.*m3.*t7.*t8;
t23 = t10.^2;
t24 = m1.*m3.*t17.*t18;
t25 = m2.*m3.*t17.*t18;
t26 = m1.*m3.*t17.*t23;
t27 = m2.*m3.*t17.*t23;
t28 = l1.*lc3.*m1.*m3.*t4.*t9;
t29 = lc1.*lc3.*m1.*m3.*t3.*t10;
t30 = I1.*m1;
t31 = I1.*m2;
t32 = I2.*m1;
t33 = I1.*m3;
t34 = I2.*m2;
t35 = I3.*m1;
t36 = I2.*m3;
t37 = I3.*m2;
t38 = I3.*m3;
t39 = m1.*m3.*t11;
t40 = m2.*m3.*t11;
t41 = m1.*m2.*t12;
t42 = m2.*m3.*t12;
t43 = l1.*l2.*m1.*m3.*t4.*2.0;
t44 = l1.*lc2.*m1.*m2.*t4.*2.0;
t45 = l2.*lc3.*m1.*m3.*t9.*2.0;
t46 = l2.*lc3.*m2.*m3.*t9.*2.0;
t47 = l1.*lc3.*m1.*m3.*t4.*t9.*2.0;
t48 = lc1.*lc3.*m1.*m3.*t3.*t10.*2.0;
t52 = l1.*lc1.*m1.*m2.*t6.*2.0;
t53 = l1.*lc1.*m1.*m3.*t6.*2.0;
t54 = l1.*lc1.*m1.*m2.*t8.*2.0;
t55 = l1.*lc1.*m1.*m3.*t8.*2.0;
t62 = l2.*lc2.*m2.*m3.*2.0;
t63 = l2.*lc1.*m1.*m3.*t4.*2.0;
t64 = lc1.*lc2.*m1.*m2.*t4.*2.0;
t65 = lc2.*lc3.*m2.*m3.*t9.*2.0;
t66 = lc1.*lc3.*m1.*m3.*t4.*t9.*2.0;
t67 = l1.*lc3.*m1.*m3.*t3.*t10.*2.0;
t49 = t13+t14+t15+t16+t19+t20+t21+t22+t24+t25+t26+t27+t30+t31+t32+t33+t34+t35+t36+t37+t38+t39+t40+t41+t42+t43+t44+t45+t46+t47+t48-t52-t53-t54-t55-t62-t63-t64-t65-t66-t67;
t50 = 1.0./t49;
t56 = l1.*l2.*m1.*m3.*t4;
t57 = l1.*lc2.*m1.*m2.*t4;
t58 = l2.*lc1.*m1.*m3.*t4;
t59 = lc1.*lc2.*m1.*m2.*t4;
t60 = lc1.*lc3.*m1.*m3.*t4.*t9;
t61 = l1.*lc3.*m1.*m3.*t3.*t10;
t68 = l2.*lc3.*m1.*m3.*t9;
t69 = l2.*lc3.*m2.*m3.*t9;
t70 = lc2.*lc3.*m2.*m3.*t9;
t71 = t24+t25+t26+t27+t28+t29-t60-t61+t68+t69-t70;
t72 = da3.*t50.*t71;
t51 = -t72+da1.*t50.*(t13+t14+t15+t16+t19+t20+t21+t22+t28+t29+t56+t57-t58-t59-t60-t61-l1.*lc1.*m1.*m2.*t6.*2.0-l1.*lc1.*m1.*m3.*t6.*2.0-l1.*lc1.*m1.*m2.*t8.*2.0-l1.*lc1.*m1.*m3.*t8.*2.0);
t73 = t72-da1.*t50.*(t13+t14+t15+t16+t19+t20+t21+t22+t28+t29-t52-t53-t54-t55+t56+t57-t58-t59-t60-t61);
Fs = [-t51.^2.*(m1.*t2.^2.*t3.*t4.*2.0-m1.*t2.*t3.*(lc2+t2.*t4).*2.0);-t73.^2.*(m3.*t9.*t10.*t17.*2.0-lc3.*m3.*t10.*(l2-lc2+lc3.*t9).*2.0)];
