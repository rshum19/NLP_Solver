function planarDiver_ctrlexp

%%%%%% Adding Custom Library
addpath('three-link/toolbox');
addpath('three-link/symbolic_function_diver')
addpath('solution')
clear all; close all; clc;
%%% System Parameters

data = load('threeLink_ornt_soln12.mat');
tF = data.soln.grid.time(end)-0.01;
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
a10 = 0*(pi/180); % link1 angle
a30 = 0*(pi/180); % link3 angle
da10 = 0*(pi/180); % link1 anglular velocity
da30 = 0*(pi/180); % link3 anglular velocity

x0 = [th20;a10;a30;da10;da30];
dth20 = Ath_fun(I1,I2,I3,a10,a30,l1,l2,lc1,lc2,lc3,m1,m2,m3)*[da10;da30];

%%%%%%%%%%%%%%%%%%%%%% ODE %%%%%%%%%%%%%%%%%%%
options = odeset('RelTol',1e-9,'AbsTol',1e-12); 
tspan = linspace(0,tF,100);
[T,X]=ode45(@(t,x)threeLink_dyn(t,x,[],d),tspan,x0,options);
[Tc,Xc] = even_sample(T,X,720);


U = d.u(T);
%%%%%%%%% For Animation and Plotting %%%%%%%%%%%

v = VideoWriter('videos/anim_FrwdSim0.avi');
open(v)

plot_results(T',X',U,2);

for i=1:size(X,1)
  theta2 = X(i,1);
  alpha1 = X(i,2);
  alpha3 = X(i,3);

ThreeLinkDiver_Draw(alpha1,theta2,alpha3,d)
drawnow


frame=getframe;
writeVideo(v,frame)
end

close(v);

soln.sim.time = T';
soln.sim.state = X';
soln.sim.control = U;

%save('solution/threeLink_FrwdSim_soln0.mat','soln')
end



