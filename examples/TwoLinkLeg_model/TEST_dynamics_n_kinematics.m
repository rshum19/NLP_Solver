%
% FILENAME: .m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% DESCRIPTION:
%

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clc; clear; close all;

%% ----------------------------------------------------------
%   DEFINE ODE PROBLEM
% -----------------------------------------------------------
% Initialize model
params = params_twoLinkLeg_model;

[tTD,dY] = flight_dynamics(2);
Vimpact = dY;

% Integrator 
t0 = 0;
tF = 2;
tspan = [t0,tF];

% Initial Condition
q0 = [0, 1.414, pi/4]';
  
dq0 = [0, Vimpact,0]';
z0 = [q0;dq0];


%% ----------------------------------------------------------
%   SOLVE
% -----------------------------------------------------------
[Tsol,Xsol] = ode45(@(t,z)twoLinkLeg_Dyn_wrap(t,z,[],params),tspan,z0);
Xsol = Xsol';

[p1,p2,p3,p4,p5] = twoLinkLeg_Kin_wrap(Tsol,Xsol,params);
%% ----------------------------------------------------------
%   DISPLAY RESULTS
% -----------------------------------------------------------

% Plots
figure
subplot(1,2,1)
plot(Tsol,Xsol(1,:))
xlabel('Time [sec]')
ylabel('X pos [m]')

subplot(1,2,2)
plot(Tsol,Xsol(2,:))
xlabel('Time [sec]')
ylabel('Y pos [m]')

figure
plot(Xsol(1,:),Xsol(2,:))
xlabel('X pos [m]')
ylabel('Y pos [m]')

p5 = [zeros(size(p5));p5];
figure
plot(p5(1,:),p5(2,:));

% Animate the results:
A.plotFunc = @(t,z)( drawInvPend2(t,z,params) );
A.speed = 0.25;
A.figNum = 101;
animate(Tsol,Xsol,A)

%% ----------------------------------------------------------
%   SAVE RESULTS
% -----------------------------------------------------------
