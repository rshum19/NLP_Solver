% Test dynamics of Acrobot robot

addpath ../../

dyn.m1 = 1;  % elbow mass
dyn.m2 = 1; % wrist mass
dyn.g = 9.81;  % gravity
dyn.l1 = 0.5;   % length of first link
dyn.l2 = 0.5;   % length of second link

t0 = 0;  tF = 10;
tspan = [t0 tF];

stepAngle = 0.2;
stepRate = (2*stepAngle)/(tF-t0);
x0 = [stepAngle; -stepAngle; -stepRate; stepRate];
u = [0 0];

[T,X] = ode45(@(t,z)acrobotDynamics(z,u,dyn),tspan,x0);

A.plotFunc = @(t,z)( drawAcrobot(T,X,dyn) );
A.speed = 0.01;
A.figNum = 101;
animate(T',X',A)
