function [U,T] = autoGen_twoLink_Energy(x,y,q2,dx,dy,dq2,l1,l2,d1,d2,m1,m2,m3,I2,I3,g)
%AUTOGEN_TWOLINK_ENERGY
%    [U,T] = AUTOGEN_TWOLINK_ENERGY(X,Y,Q2,DX,DY,DQ2,L1,L2,D1,D2,M1,M2,M3,I2,I3,G)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    26-Feb-2016 03:28:55

t2 = cos(q2);
t3 = l2.^2;
t4 = sin(q2);
t5 = l1.^2;
t8 = l1.*l2.*t2.*2.0;
t6 = t3+t5-t8;
t9 = 1.0./sqrt(t6);
t10 = t4.^2;
t11 = 1.0./t6.^(3.0./2.0);
t7 = dx+dq2.*(d1.*l2.*t2.*t9-d1.*l1.*t3.*t10.*t11);
t12 = dx+dq2.*(d2.*l1.*t2.*t9-d2.*l2.*t5.*t10.*t11);
t26 = l2.*t2;
t13 = l1-t26;
t16 = 1.0./t6;
t17 = t16.^(3.0./2.0);
t19 = l1.*l2;
t20 = t2.*t5;
t21 = t2.*t3;
t22 = t2.^2;
t23 = l1.*l2.*t22;
t24 = t19-t20-t21+t23;
t14 = dy-(d1.*dq2.*t3.*t4.*t17.*t24.*sign(t13))./t13;
t15 = dq2.^2;
t27 = l1.*t2;
t18 = l2-t27;
t25 = dy+dq2.*(l1.*l2.*t4.*t9+(d2.*t4.*t5.*t17.*t24.*sign(t18))./t18);
U = I2.*t15.*(1.0./2.0)+I3.*t15.*(1.0./2.0)+m2.*t7.^2.*(1.0./2.0)+m3.*t12.^2.*(1.0./2.0)+m2.*t14.^2.*(1.0./2.0)+m3.*t25.^2.*(1.0./2.0)+m1.*(dx.^2+dy.^2).*(1.0./2.0);
if nargout > 1
    t28 = sqrt(t16);
    T = g.*m2.*(y-d1.*t28.*abs(t13))+g.*m1.*y+g.*m3.*(y+sqrt(t6)+d2.*t28.*abs(t18));
end
