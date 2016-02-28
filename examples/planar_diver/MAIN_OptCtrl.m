% PURPOSE:  
% FILENAME: .m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%------------------- Instructions ---------------------------

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
% Clear workspace
clc; clear; close all;

% Add paths
addpath('../../');
addpath('../../methods');
addpath('wrapperFncs');
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_PlanarDiver;
OCP.model.dynamics = @(t,x,u)planarDiverDynamicsWrapper(t,x,u);

%% ----------------------------------------------------------
%   DEFINE OCP PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
%OCP.costFnc = @(t,x,u)invPend_CostFnc(t,x,u, OCP.model.params);
%OCP.pathCostFnc = @(t,x,u)(u.^2);
OCP.pathCostFnc = [];
OCP.bndCostFnc = @(t,x,u)planarDiver_bndCostFnc(t,x,u);

% INITIAL GUESS
% ------------------------
% Set the type of initial guess:
%   linear: Just define a intial and final state and OptCtrlSolver will
%           make a linear interpolation between the two points
%   custom: The user provides a custom initial guess "shape". 
%           NOTE: custom initial guess has to be descritized with the same
%           size than OCP.options.nGrid
OCP.options.IGtype = 'linear';

% Time span
t0 = 0;
tF = 2.5;
OCP.ig.time = [t0,tF];

% State 
% q = [a1; a2; th; da1; da2; dth]
stepAngle = 0.2;
stepRate = (2*stepAngle)/(tF-t0);
x0 = [0; 0; pi; 0; 0; 0];
xF = [0; 0; 3*pi; 0; 0; 0];
OCP.ig.state = [x0,xF];

% Control
u0 = [0;0];
uF = [0;0];
OCP.ig.control = [u0,uF];

% CONSTRAINTS & BOUNDARIES
% ------------------------
% Nonlinear constraints:
%   OCP.pathCst = @(t,x,u)pathCst(z);
%   OCP.bndCst = @(t0,x0,u0,tF,xF,uF)bndCst(z);
OCP.pathCst = [];
OCP.bndCst = [];

% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = t0+0.1;
OCP.bounds.finalTime.ub = tF*2;

% State:
% q = [a1; a2; th; da1; da2; dth]
OCP.bounds.state.lb = [-2*pi; -2*pi; -inf(4,1)];
OCP.bounds.state.ub = [2*pi; 2*pi; inf(4,1)];
OCP.bounds.initState.lb =  [0; 0; pi/2; 0; 0; 0];
OCP.bounds.initState.ub =  [0; 0; pi/2; 0; 0; 0];
OCP.bounds.finalState.lb = [-2*pi; -2*pi; 3*pi; zeros(3,1)];
OCP.bounds.finalState.ub = [2*pi; 2*pi; 3*pi; zeros(3,1)];

maxAngVel = 300; 
OCP.bounds.control.lb = [-maxAngVel; -maxAngVel];
OCP.bounds.control.ub = [maxAngVel; maxAngVel];

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------

% method = 'euler';
method = 'trapezoidal';
% method = 'hermiteSimpson';

OCP.options.method = method;
OCP.options.nGrid = 25;

% For a full list of options refer to :
%   http://www.mathworks.com/help/optim/ug/fmincon.html#inputarg_options

OCP.options.fminOpt.MaxFunEval = 1e5;

%% ----------------------------------------------------------
%   SOLVE NLP PROBLEM
% -----------------------------------------------------------

soln = OptCtrlSolver(OCP);

tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);
%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------
params = OCP.model.params;

figure
subplot(4,1,1)
plot(t,z(1:2,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angle [rad]');
title('Joint angle position')

subplot(4,1,2)
plot(t,z(3,:));
legend('\theta, orient')
xlabel('Time [sec]');
ylabel('Angle [rad]');
title('Body orientation')

subplot(4,1,3)
plot(t,z(4:5,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angular Velocites [rad/s]');

subplot(4,1,4)
plot(t,u(1,:),t,u(1,:))
xlabel('Time [sec]');
ylabel('U control input');

% Animate the results:
A.plotFunc = @(t,z)( drawPlanarDiver(t,z,params) );
A.speed = 0.25;
A.figNum = 101;
animate(t,z,A)

%%% TODO
% Draw a stop-action animation:
