function G = G_fun(a1,a3,g,l1,l2,lc1,lc2,lc3,m1,m2,m3,th2)
%G_FUN
%    G = G_FUN(A1,A3,G,L1,L2,LC1,LC2,LC3,M1,M2,M3,TH2)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    26-Feb-2016 22:57:22

t2 = sin(th2);
t3 = a1+th2;
t4 = sin(t3);
t5 = l1-lc1;
t6 = a3-th2;
t7 = sin(t6);
G = [0.0;g.*m1+g.*m2+g.*m3;-g.*m1.*(lc2.*t2+t4.*t5)-g.*m3.*(lc3.*t7-t2.*(l2-lc2));-g.*m1.*t4.*t5;g.*lc3.*m3.*t7];
