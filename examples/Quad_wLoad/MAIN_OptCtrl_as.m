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
add_SNOPT;
addpath('../../');
addpath('../../methods');
addpath('sim_quad_spring/GeoControl-Toolbox');
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_quad_wload;
OCP.model.dynamics = @(t,x,u)odefun_dynamics(t,x,u,OCP.model.params);

%% ----------------------------------------------------------
%   DEFINE OCP PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
%OCP.costFnc = @(t,x,u)invPend_CostFnc(t,x,u, OCP.model.params);
%OCP.pathCostFnc = @(t,x,u)(u.^2);
OCP.pathCostFnc = @(t,x,u)(u.^2);
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
tF = 5;
OCP.ig.time = [t0,tF];

% State 
% dq = [xL_dot; vL_dot; q_dot; omega_dot; reshape(R_dot,
% 9,1);Omega_dot;l_dot; dl_dot];  26x1
xL = [0;0;0];%zeros(3,1) ;
vL = zeros(3,1) ;
q = -[0;0;1];
omega = [0;0;0];
R = eye(3,3) ;
Omega = [0;0;0] ;
l = 1 ;%- (data.params.mL*data.params.g)/data.params.k;
dl = 0;

x0 = [xL; vL; q; omega; reshape(R, 9,1);Omega];%l;dl];
xF = [[1;0;0]; vL; q; omega; reshape(R, 9,1);Omega];%l;dl];
OCP.ig.state = [x0,xF];

% Control
u0 = zeros(4,1);
uF = zeros(4,1);
OCP.ig.control = [u0,uF];

% CONSTRAINTS & BOUNDARIES
% ------------------------
% Nonlinear constraints:
%   OCP.pathCst = @(t,x,u)pathCst(z);
%   OCP.bndCst = @(t0,x0,u0,tF,xF,uF)bndCst(z);
OCP.pathCst = @(t,x,u)pathCst(t,x,u,OCP.model.params);
OCP.bndCst = [];

% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = tF;
OCP.bounds.finalTime.ub = tF;

% State:
% q = xL; vL; q; omega; reshape(R, 9,1);Omega;l;dl];
OCP.bounds.state.lb = [-1;0;0; -5*ones(3,1); -ones(3,1); -5*ones(3,1); -ones(9,1); -5*ones(3,1)];% 0; -inf];
OCP.bounds.state.ub = [2;0;0; 5*ones(3,1); ones(3,1); 5*ones(3,1); ones(9,1); 5*ones(3,1)];% 1; inf];
OCP.bounds.initState.lb =  [xL; vL; q; omega; reshape(R, 9,1);Omega];%l;dl];
OCP.bounds.initState.ub =  [xL; vL; q; omega; reshape(R, 9,1);Omega];%l;dl];
OCP.bounds.finalState.lb = [[1;0;0]; vL; q; omega; reshape(R, 9,1);Omega];%l;dl];
OCP.bounds.finalState.ub = [[1;0;0]; vL; q; omega; reshape(R, 9,1);Omega];%l;dl];

% Control:
maxAngVel = 300*10; 
OCP.bounds.control.lb = [0; -100*ones(3,1)];
OCP.bounds.control.ub = [50; 100*ones(3,1)];

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
options(1).method = 'euler_back';
options(1).nGrid = 15;
options(1).fminOpt = fminOpt;

% % %--- Interation 2
% options(2).method = 'euler_back';
% options(2).nGrid = 15;
% options(2).fminOpt = fminOpt;
% 
% % %--- Interation 3
% options(3).method = 'euler_back';
% options(3).nGrid = 15;
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
% solnFolderName = 'solution';
% fileName = 'threeLink_ornt_soln';
% overWrite = 0;
% Notes = 'Without drift test for 3pi';
% saveResults(solnFolderName, fileName, overWrite, soln,OCP,Notes)
%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------
%plot_results(t,z,u,2)

