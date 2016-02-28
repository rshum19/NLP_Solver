function C = C_fun(a1,a3,da1,da3,dth2,dx,dy,l1,l2,lc1,lc2,lc3,m1,m3)
%C_FUN
%    C = C_FUN(A1,A3,DA1,DA3,DTH2,DX,DY,L1,L2,LC1,LC2,LC3,M1,M3)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    26-Feb-2016 22:57:22

t2 = sin(a1);
t3 = l1-lc1;
t4 = sin(a3);
t5 = cos(a1);
t6 = cos(a3);
t7 = da1.*m1.*t2.*t3;
t8 = t7-da3.*lc3.*m3.*t4;
t9 = -da3.*lc3.*m3.*t6-da1.*m1.*t3.*t5;
t10 = t3.^2;
t11 = m1.*t2.*t5.*t10.*2.0;
t12 = t3.*t5;
t13 = lc2+t12;
t16 = m1.*t2.*t3.*t13.*2.0;
t14 = t11-t16;
t15 = da1.*t14;
t17 = lc3.^2;
t18 = m3.*t4.*t6.*t17.*2.0;
t19 = lc3.*t6;
t20 = l2-lc2+t19;
t23 = lc3.*m3.*t4.*t20.*2.0;
t21 = t18-t23;
t22 = da3.*t21;
t24 = dth2.*m1.*t2.*t3;
t25 = dth2.*t14;
t26 = dx.*m1.*t2.*t3;
t27 = dth2.*t21;
C = reshape([0.0,0.0,t8,-t24,dth2.*lc3.*m3.*t4,0.0,0.0,t9,dth2.*m1.*t3.*t5,dth2.*lc3.*m3.*t6,t8,t9,t15+t22,-t25-t26+dy.*m1.*t3.*t5,-t27+dx.*lc3.*m3.*t4+dy.*lc3.*m3.*t6,t24+da1.*m1.*t2.*t3.*2.0,da1.*m1.*t3.*t5.*-2.0-dth2.*m1.*t3.*t5,t15+t25+t26-dy.*m1.*t3.*t5,0.0,0.0,da3.*lc3.*m3.*t4.*2.0-dth2.*lc3.*m3.*t4,da3.*lc3.*m3.*t6.*2.0-dth2.*lc3.*m3.*t6,-t22+t27-dx.*lc3.*m3.*t4-dy.*lc3.*m3.*t6,0.0,0.0],[5,5]);