function ThreeLinkDiver_expFull
clear all; clc;
%%%%%% Adding Custom Library
addpath('toolbox');
addpath('symbolic_function_diver')
addpath('../solution')
%%% System Parameters

data = load('threeLink_ornt_soln12.mat');

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

x0 = 0;
y0 = 15;
th20 = 0*(pi/180); % body angle
a10 = 0*(pi/180); % link1 angle
a30 = 0*(pi/180); % link3 angle

dx0 = 0;
dy0 = 0;
dth20 = 2;
da10 = 0*(pi/180); % link1 anglular velocity
da30 = 0*(pi/180); % link3 anglular velocity

z0 = [x0;y0;th20;a10;a30;dx0;dy0;dth20;da10;da30];
dth20 = Ath_fun(I1,I2,I3,a10,a30,l1,l2,lc1,lc2,lc3,m1,m2,m3)*[da10;da30];

%%%%%%%%%%%%%%%%%%%%%% ODE %%%%%%%%%%%%%%%%%%%

options = odeset('Events',@events_diver,'RelTol',1e-9,'AbsTol',1e-12);

[T,X]=ode45(@(t,z)threeLink_FullDyn(t,z,d),[0 2*2.4],z0,options);
[Tc,Xc] = even_sample(T,X,720);


U = d.u(T);
%%%%%%%%% For Animation and Plotting %%%%%%%%%%%

%  v = VideoWriter('videoRedux.avi');
%  open(v)

% plot_results(T',X',U,2);
X = X';
figure(2)
subplot(3,1,1)
plot(T,X(1,:))
title('X-position')

subplot(3,1,2)
plot(T,X(2,:))
title('Y-position')

subplot(3,1,3)
plot(T,X(3,:))
title('\Theta-position')

figure(3)
subplot(2,1,1)
plot(T,X(4,:))
title('\alpha_1-position')

subplot(2,1,2)
plot(T,X(5,:))
title('\alpha_3-position')
% for i=1:size(Xc,1)
%   theta2 = Xc(i,1);
%   alpha1 = Xc(i,2);
%   alpha3 = Xc(i,3);
% 
% ThreeLinkDiver_Draw(alpha1,theta2,alpha3,d)
% drawnow
% % frame=getframe;
% % writeVideo(v,frame)
% end

function [DZ] = threeLink_FullDyn(t,z,d)

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
g = d.g;

x = z(1);
y = z(2);
th2 = z(3);
a1 = z(4);
a3 = z(5);

dx = z(6);
dy = z(7);
dth2 = z(8);
da1 = z(9);
da3 = z(10);

dz = [dx;dy;dth2;da1;da3];

%dth2=Ath_fun(I1,I2,I3,a1,a3,l1,l2,lc1,lc2,lc3,m1,m2,m3)*[da1;da3];
%Js = Js_fun(I1,I3,a1,a3,l1,lc1,lc3,m1,m3);
%Fs = Fs_fun(I1,I2,I3,a1,a3,da1,da3,l1,l2,lc1,lc2,lc3,m1,m2,m3);
D = D_fun(I1,I2,I3,a1,a3,l1,l2,lc1,lc2,lc3,m1,m2,m3);
C = C_fun(a1,a3,da1,da3,dth2,dx,dy,l1,l2,lc1,lc2,lc3,m1,m3);
G = G_fun(a1,a3,g,l1,l2,lc1,lc2,lc3,m1,m2,m3,th2);
B = [zeros(3,2);eye(2)];


ddz = D\(-C*dz - G);
if t >= 2.5
    ddz(4:5,:) = zeros(2,1);
else
    U = d.u(t);
    ddz(4,:) = U(1)/2;
    ddz(5,:) = U(2)/2;
end
DZ = [dz;ddz];

% --------------------------------------------------------------
function [value,isterminal,direction] = events_diver(t,z)
% Locate the time when height passes through zero in a 
% decreasing direction and stop integration.
value = z(2);     % Detect height = 0
isterminal = 1;   % Stop the integration
direction = -1;   % Negative direction only



