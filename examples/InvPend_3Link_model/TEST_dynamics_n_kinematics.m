%INVPEND_DYNAMICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% DESCRIPTION:
%

%% ----------------------------------------------------------
%   DEFINE ODE PROBLEM
% -----------------------------------------------------------
% Initialize model
params = params_3link_invPend;

% Integrator 
t0 = 0;
tF = 2;
tspan = [t0,tF];

% Initial Condition
q0 = [-2; -2; -2];
dq0 = zeros(3,1) + ones(3,1)*4;
z0 = [q0;dq0]';
u = [];
%% ----------------------------------------------------------
%   SOLVE
% -----------------------------------------------------------

[Tsol,Xsol] = ode45(@(t,z)invPend_Dynamics(t,z,u,params),tspan,z0);
Xsol = Xsol';

%% ----------------------------------------------------------
%   DISPLAY RESULTS
% -----------------------------------------------------------

% Plots
figure
subplot(2,1,1)
plot(Tsol,Xsol(1:3,:));
legend('joint 1','joint 2','joint 3')
xlabel('Time [sec]');
ylabel('Angle [rad]');

subplot(2,1,2)
plot(Tsol,Xsol(4:6,:));
legend('joint 1','joint 2','joint 3')
xlabel('Time [sec]');
ylabel('Angle rate [rad/sec]');

% Animate the results:
A.plotFunc = @(t,z)( drawInvPend3(t,z,params) );
A.speed = 0.25;
A.figNum = 101;
animate(Tsol,Xsol,A)

