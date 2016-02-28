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
addpath('three-link');
addpath(genpath('three-link'));
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = ThreeLinkDiver_Properties;
OCP.model.dynamics = @(t,x,u)threeLink_dyn_wrap(t,x,u,OCP.model.params);

%% ----------------------------------------------------------
%   DEFINE OCP PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
%OCP.costFnc = @(t,x,u)invPend_CostFnc(t,x,u, OCP.model.params);
%OCP.pathCostFnc = @(t,x,u)(u.^2);
OCP.pathCostFnc =@(t,x,u)(u.^2);
OCP.bndCostFnc =[];% @(t,x,u)planarDiver_bndCostFnc(t,x,u);

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
% q = [th; a1; a2; da1; da2]
x0 = [0; 0; 0; 0; 0];
xF = [0; 0; 0; 0; 0];
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
OCP.bounds.finalTime.ub = tF;

% State:
% q = [th; a1; a2; th; da1; da2]
OCP.bounds.state.lb = [-inf; 0; 0; -inf(2,1)];
OCP.bounds.state.ub = [inf; pi; 2*pi/3; inf(2,1)];
OCP.bounds.initState.lb =  [0; 0; 0; 0; 0];
OCP.bounds.initState.ub =  [0; 0; 0; 0; 0];
OCP.bounds.finalState.lb = [3*pi; 0; 0; zeros(2,1)];
OCP.bounds.finalState.ub = [3*pi; 0*pi; 0*2*pi/3; zeros(2,1)];

maxAngVel = 300*5; 
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
fminOpt = optimoptions('fmincon','Display','iter','Algorithm','interior-point','MaxIter',3e3,'MaxFunEvals',1e6,'TolFun',1e-5,'TolX',1e-5);
OCP.options.fminOpt = fminOpt;

%--- Interation 1
options(1).method = 'trapezoidal';
options(1).nGrid = 15;
options(1).fminOpt = fminOpt;

% % %--- Interation 2
% options(2).method = 'trapezoidal';
% options(2).nGrid = 50;
% options(2).fminOpt = fminOpt;
% 
% % %--- Interation 3
% options(3).method = 'trapezoidal';
% options(3).nGrid = 75;
% options(3).fminOpt = fminOpt;

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

tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);

% Save results
solnFolderName = 'solution';
fileName = 'threeLink_ornt_soln';
overWrite = 0;
Notes = 'With drift test for 3pi';
saveResults(solnFolderName, fileName, overWrite, soln,OCP,Notes)
%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------
params = OCP.model.params;

plot_results(t,z,u,2)


% Animate the results:
% A.plotFunc = @(t,z)( drawPlanarDiver(t,z,params) );
% A.speed = 0.25;
% A.figNum = 101;
% animate(t,z,A)

v = VideoWriter('anim_OptSoln0.avi');
open(v)
for i=1:size(z,2)
ThreeLinkDiver_Draw(z(2,i),z(1,i),z(3,i),OCP.model.params)
drawnow
frame=getframe;
writeVideo(v,frame)
pause(0.01);
end

close(v);
