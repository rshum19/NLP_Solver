function ThreeLinkDiver_Reduced3

%%%%%% Adding Custom Library
addpath('toolbox');
addpath('symbolic_function_diver')
addpath('../solution')
addpath('../../..');
%%% System Parameters

data = load('threeLink_ornt_soln9.mat');

d = ThreeLinkDiver_Properties();
d.u = data.soln(end).interp.control;
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

%%% Desired Configs

d.th2d = 90*(pi/180);
d.a1d = 20*(pi/180);
d.a3d = -30*(pi/180);
d.pd = 1 ;

%%% Initial Conditions

th20 = 0*(pi/180); % body angle
a10 = 40*(pi/180); % link1 angle
a30 = -10*(pi/180); % link3 angle
da10 = 0*(pi/180); % link1 anglular velocity
da30 = 0*(pi/180); % link3 anglular velocity

x0 = [th20;a10;a30;da10;da30];
dth20 = Ath_fun(I1,I2,I3,a10,a30,l1,l2,lc1,lc2,lc3,m1,m2,m3)*[da10;da30];

%%%%%%%%%%%%%%%%%%%%%% ODE %%%%%%%%%%%%%%%%%%%
options = odeset('RelTol',1e-9,'AbsTol',1e-12); 
[T,X]=ode45(@threeLink_dyn,[0 3],x0,options,d);
[Tc,Xc] = even_sample(T,X,720);


%%%%%%%%% For Animation and Plotting %%%%%%%%%%%

%  v = VideoWriter('videoRedux.avi');
%  open(v)

for i=1:size(Xc,1)
  theta2 = Xc(i,1);
  alpha1 = Xc(i,2);
  alpha3 = Xc(i,3);

ThreeLinkDiver_Draw(alpha1,theta2,alpha3,d)
drawnow


% frame=getframe;
% writeVideo(v,frame)
end

% close(v)



Fs=Fs';


figure
plot(T, U)
title('Input v/s Time')
xlabel('time (s)')
ylabel('input (u)')
legend('u_{\theta}','u_{\alpha}')
axis('tight')

plot_results(t1,z1,u1,1);

end


function [dx,u] = threeLink_dyn(t,x,d)


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


dth2=Ath_fun(I1,I2,I3,a1,a3,l1,l2,lc1,lc2,lc3,m1,m2,m3)*[da1;da3];
Js = Js_fun(I1,I3,a1,a3,l1,lc1,lc3,m1,m3);
Fs = Fs_fun(I1,I2,I3,a1,a3,da1,da3,l1,l2,lc1,lc2,lc3,m1,m2,m3);

dx(1) = Ath_fun(I1,I2,I3,a1,a3,l1,l2,lc1,lc2,lc3,m1,m2,m3)*[da1;da3];
dx(2) = da1;
dx(3) = da3;

B = eye(2);

u = d.u(t);
% u1 = interp1(d.soln.time,d.soln.control(1,:),t);
% u2 = interp1(d.soln.time,d.soln.control(2,:),t);

dx(4) = u(1);
dx(5) = u(2);

dx = dx';

end


