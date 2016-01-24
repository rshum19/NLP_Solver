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
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_3link_invPend;
OCP.model.dynamics = @(t,x,u)invPend_Dynamics(t,x,u,OCP.model.params);

%% ----------------------------------------------------------
%   DEFINE OCP PROBLEM PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
%%% TODO
% change to running and terminal cost
OCP.pathCostFnc = @(t,x,u)invPend_costFnc(t,x,u,OCP.model.params);
OCP.bndCostFnc = [];

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
tF = 5;
OCP.ig.time = [t0,tF];

% State 
stepAngle = 0.2;
stepRate = (2*stepAngle)/(tF-t0);
x0 = [-2; -2; -2; -stepRate; -stepRate; -stepRate];
xF = [0;0;0; stepRate; stepRate; stepRate];
OCP.ig.state = [x0,xF];

% Control
u0 = 0;
uF = 0;
OCP.ig.control = [u0,uF; u0,uF];

% CONSTRAINTS & BOUNDARIES
% ------------------------
 
% Nonlinear constraints
%OCP.pathCst = @(t,x,u)pathCst(z);
%OCP.bndCst = @(t0,x0,u0,tF,xF,uF)bndCst(z);
OCP.pathCst = [];
OCP.bndCst = [];

% You can let time to be free by not setting any bounds on the final time
% Set the finalTime .lb = .ub to have a fixed final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = t0*2;
OCP.bounds.finalTime.ub = tF*5;

% State:
%   [q1; q2; dq1; dq2];
%  set starting position by setting initState .lb = .ub
%  desired final postionby setting finalState .lb = .ub
OCP.bounds.state.lb = [-2*pi; -2*pi; -2*pi; -inf(3,1)]; % Change to so they are set from the model parameters
OCP.bounds.state.ub = [2*pi; 2*pi; 2*pi; inf(3,1)];
OCP.bounds.initState.lb = [pi; pi; pi; 0; 0; 0];
OCP.bounds.initState.ub = [pi; pi; pi; 0; 0; 0];
OCP.bounds.finalState.lb = zeros(6,1);
OCP.bounds.finalState.ub = zeros(6,1);

maxTorque = 25; 
OCP.bounds.control.lb = [-maxTorque; -maxTorque]; % Change so they are set from model parametes
OCP.bounds.control.ub = [maxTorque; maxTorque];

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------

% method = 'euler';
method = 'trapezoidal';
% method = 'hermiteSimpson';

OCP.options.method = 'trapezoidal';
OCP.options.nGrid = 15;

% For a full list of options refer to :
%   http://www.mathworks.com/help/optim/ug/fmincon.html#inputarg_options

OCP.options.fminOpt.MaxFunEval = 1e5;

%% ----------------------------------------------------------
%   SOLVE NLP PROBLEM
% -----------------------------------------------------------

soln = OptCtrlSolver(OCP);

tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
x = soln(end).interp.state(t);
u = soln(end).interp.control(t);

%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------

figure
subplot(3,1,1)
plot(t,x(1:3,:));
legend('joint 1','joint 2','joint 3')
xlabel('Time [sec]');
ylabel('Angle [rad]');

subplot(3,1,2)
plot(t,x(4:6,:));
legend('joint 1','joint 2','joint 3')
xlabel('Time [sec]');
ylabel('Angle rate [rad/sec]');

subplot(3,1,3)
plot(t,u)
legend('joint 2', 'joint 3')
xlabel('Time [sec]');
ylabel('U control input');

% Animate the results:
A.plotFunc = @(t,x)drawInvPend3(t,x,OCP.model.params);
A.speed = 0.25;
A.figNum = 101;
animate(t,x,A)