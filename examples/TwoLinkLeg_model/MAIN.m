% PURPOSE: Performance a contact invariant optimization of a box
%          with 3 contact points falling on a slope
% FILENAME: MAIN.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 2/16/16
%------------------- Instructions ---------------------------

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
% Clear workspace
clc; clear; close all;

% Define folder names
subFolderName = '';

% Add paths
addpath('../../');
addpath('../../methods');
%addpath(solnFolderName );
%addpath(subFolderName);
addpath(fullfile(subFolderName,'autogenFncs'));
addpath(fullfile(subFolderName,'wrapperFncs'));
%addpath(fullfile(subFolderName,'helperFncs'));
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_twoLinkLeg_model;
OCP.model.dynamics = @(t,x,u)twoLinkLeg_Dyn_wrap(t,x,u,OCP.model.params);

%% ----------------------------------------------------------
%   DEFINE OCP PROBLEM PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
% e.g.
%   Step cost: OCP.pathCostFnc = @(t,x,u)model_StepCostFnc(t,x,u, OCP.model.params);
%   Terminal cost: OCP.bndCostFnc = @(t,x,u)model_TerminalcostFnc(t,x,u, OCP.model.params);
OCP.pathCostFnc = @(t,x,u)(u.^2);
OCP.bndCostFnc = @(t,x,u)twoLinkLeg_bndCostFnc(t,x,u,OCP.model.params);

% INITIAL GUESS
% ------------------------

% Load initial guess

% Time span
t0 = 0;
tF = 2.5;
OCP.ig.time = [t0,tF];
[tTD,dY] = flight_dynamics(2);
Vimpact = dY;
% State 
% x0, xF:   state vector [q;dq] => [2*n X 1] column vector
%   [x; y; q1; q2; dx; dy; dq1; dq2];
OCP.ig.state = [0, 0;...
                1.414, 1.414;...
                pi/4, pi/4;...
                -pi/2, -pi/2;...
                0,0;
                Vimpact,
                zeros(2,2)];

% Control input
OCP.ig.control = [0,0];

% CONSTRAINTS & BOUNDARIES
% ------------------------
 
%----- Nonlinear constraints
% e.g.
% Path:
%   OCP.pathCst = @(t,x,u)pathCst(z);
% Boundary: 
%   OCP.bndCst = @(t0,x0,u0,tF,xF,uF)bndCst(z);
% Complementary:
%   OCP.compCst = @(t0,x0,u0,tF,xF,uF)compCst(z);
OCP.pathCst = [];
OCP.bndCst = [];

%----- Linear constraints
% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = tF;
OCP.bounds.finalTime.ub = tF;

% State:
OCP.bounds.state.lb = [0; 0; 0; -3*pi/2; -inf(4,1)]; 
OCP.bounds.state.ub = [0; 2; 3*pi/2; 0; inf(4,1)];
OCP.bounds.initState.lb = [0; 0.5; 0; -3*pi/2; 0; Vimpact; zeros(2,1)];
OCP.bounds.initState.ub = [0; 2; 3*pi/2; 0; 0;  Vimpact; zeros(2,1)];
OCP.bounds.finalState.lb = [0; 0.5; 0; -3*pi/2; zeros(4,1)];
OCP.bounds.finalState.ub = [0; 2; 3*pi/2; 0; zeros(4,1)];

% Control:
% Change so they are set from model parametes
maxTau = 500;
OCP.bounds.control.lb = -500; 
OCP.bounds.control.ub = 500;

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------
%%% TODO
% add capability for other methods
% method = 'euler';
% method = 'euler_mod';
% method = 'euler_back';
% method = 'trapezoidal';
% method = 'hermiteSimpson';

% Fmincon options:
% for a full list of options refer to :
%   http://www.mathworks.com/help/optim/ug/fmincon.html#inputarg_options
fminOpt = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxIter',3e3,'MaxFunEvals',1e4,'TolFun',1e-6);

%--- Interation 1
options(1).method = 'trapezoidal';
options(1).nGrid = 20;
options(1).fminOpt = fminOpt;
options(1).fminOpt.TolCon = 0.01;

% %--- Interation 2
% options(2).method = 'euler_mod';
% options(2).nGrid = 50;
% options(2).fminOpt = fminOpt;
% options(2).fminOpt.MaxFunEvals = 5e4;

% %--- Interation 3
%options(3).method = 'euler_mod';
%options(3).nGrid = 50;
%options(3).fminOpt = fminOpt;
%options(3).fminOpt.MaxFunEvals = 5e6;

% Display initial guess
%displayIGnBnds(OCP.ig,OCP.bounds,options(1).nGrid);
%% ----------------------------------------------------------
%   SOLVE NLP PROBLEM
% -----------------------------------------------------------
soln(length(options)) = struct('info',[],'grid',[],'interp',[],'guess',[],'time',[]);
for iter = 1:size(options,2)
    fprintf('--------- Optimization Pass No.: %d ---------\n',iter)
    % Set options to pass to solver
    OCP.options = options(iter);
    
    % Solve Optimal control problem
    tic;
    soln(iter) = OptCtrlSolver(OCP);
    time = toc;
    
    % save time of optimization
    soln(iter).time = time;
    
    % Update initial condition
    OCP.ig = soln(iter).grid;
end

t = soln(end).grid.time;
z = soln(end).grid.state;
u = soln(end).grid.control;
lambda = soln(end).grid.lambda;
guess = soln(end).guess;

% Save results
% fileName = 'fallingBox_slanted_soln';
% overWrite = 1;
% Notes = 'Phi(x,y,theta), 1 contact point';
% saveResults(solnFolderName, fileName, overWrite, soln,OCP,Notes)
%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------

% Plot results
%fallingBox_plotResults(t,z,lambda)

% Animate the results:
% A.plotFunc = @(t,z)( drawModel(t,z,dyn) );
% A.speed = 0.25;
% A.figNum = 101;
% animate(t,z,A)

%%% TODO
% Draw a stop-action animation:
