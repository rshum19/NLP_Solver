% PURPOSE:  
% FILENAME: .m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%------------------- Instructions ---------------------------

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------

% Clear workspace
clc; clear;

% Add paths
addpath('../../');
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = model_params;
OCP.model.dynamics = @(t,x,u)model_Dynamics(t,x,u,OCP.model.params);

%% ----------------------------------------------------------
%   DEFINE OCP PROBLEM PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
OCP.pathCostFnc = @(t,x,u)model_pathcostFnc(t,x,u, OCP.model.params);
OCP.bndCostFnc = @(t,x,u)model_bndcostFnc(t,x,u, OCP.model.params);

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
stepAngle = 0.2;
stepRate = (2*stepAngle)/(tF-t0);
x0 = [];
xF = [];
OCP.ig.state = [x0,xF];

% Control
u0 = 0;
uF = 0;
OCP.ig.control = [u0,uF];

% CONSTRAINTS & BOUNDARIES
% ------------------------
 
% Nonlinear constraints
%OCP.pathCst = @(t,x,u)pathCst(z);
%OCP.bndCst = @(t0,x0,u0,tF,xF,uF)bndCst(z);
OCP.pathCst = [];
OCP.bndCst = [];

% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = tF;
OCP.bounds.finalTime.ub = tF;

% State:
%   [q1; q2; dq1; dq2];
%  
OCP.bounds.state.lb = []; 
OCP.bounds.state.ub = [];
OCP.bounds.initState.lb = [];
OCP.bounds.initState.ub = [];
OCP.bounds.finalState.lb = [];
OCP.bounds.finalState.ub = [];

% Control:
OCP.bounds.control.lb = []; 
OCP.bounds.control.ub = [];

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------

%%% TODO
% add capability for other methods
% method = 'euler';
method = 'trapezoidal';
% method = 'hermiteSimpson';

OCP.options.method = 'trapezoidal';
OCP.options.nGrid = 15;

% For a full list of options refer to :
%   http://www.mathworks.com/help/optim/ug/fmincon.html#inputarg_options
%%% TODO setting options here is not working need to see why
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
dyn = OCP.model.params;

figure
subplot(3,1,1)
plot(t,z(1:2,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angle [rad]');

subplot(3,1,2)
plot(t,z(3:4,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angle rate [rad/sec]');

subplot(3,1,3)
plot(t,u)
xlabel('Time [sec]');
ylabel('U control input');

% Animate the results:
A.plotFunc = @(t,z)( drawModel(t,z,dyn) );
A.speed = 0.25;
A.figNum = 101;
animate(t,z,A)

%%% TODO
% Draw a stop-action animation:
