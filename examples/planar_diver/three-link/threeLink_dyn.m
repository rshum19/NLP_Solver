function [dx] = threeLink_dyn(t,x,u,d)

gen_mom_th = 48.0851/2.5*0;
l1=d.l1;
l2=d.l2;
l3=d.l3;
lc1 = d.lc1;
lc2 = d.lc2;
lc3 = d.lc3;
m1=d.m1;
m2=d.m2;
m3=d.m3;
I1=d.I1;
I2=d.I2;
I3=d.I3;

th2 = x(1);
a1 = x(2);
a3 = x(3);
da1 = x(4);
da3 = x(5);


%dth2=Ath_fun(I1,I2,I3,a1,a3,l1,l2,lc1,lc2,lc3,m1,m2,m3)*[da1;da3];
%Js = Js_fun(I1,I3,a1,a3,l1,lc1,lc3,m1,m3);
%Fs = Fs_fun(I1,I2,I3,a1,a3,da1,da3,l1,l2,lc1,lc2,lc3,m1,m2,m3);

dx(1) = Ath_fun(I1,I2,I3,a1,a3,l1,l2,lc1,lc2,lc3,m1,m2,m3)*[da1;da3] + Drift_theta_fun(I1,I2,I3,a1,a3,0,0,gen_mom_th,l1,l2,lc1,lc2,lc3,m1,m2,m3);
dx(2) = da1;
dx(3) = da3;

%B = eye(2);
%u = threeLink_con(th2,a1,a3,dth2,da1,da3,Js,Fs,B,d);
%out = Js\(-Fs + B*u);

if isempty(u)
    u = d.u(t);
end

dx(4) = u(1);
dx(5) = u(2);

dx = dx';

% y = [t x' u'];
% disp(y)
end