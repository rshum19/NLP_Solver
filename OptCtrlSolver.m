function [ Soln ] = OptCtrlSolver( OCP )
%OPTCTRLSOLVER solves a user defined Optimal Control Problem 
%   Detailed explanation goes here


%% ----------------------------------------------------------
%   READ INPUT
% -----------------------------------------------------------
dynamics = OCP.model.dynamics;
IG = OCP.ig;
Bnds = OCP.bounds;
OPT = OCP.options;
nGrid = OCP.options.nGrid;

%% ----------------------------------------------------------
%   FORMAT INITIAL GUESS
% -----------------------------------------------------------
% Interpolate the guess at the grid-points for transcription:
guess.tSpan = IG.time([1,end]);
guess.time = linspace(guess.tSpan(1), guess.tSpan(2), nGrid);
guess.state = interp1(IG.time', IG.state', guess.time')';
guess.control = interp1(IG.time', IG.control', guess.time')';

% Mux intial guess
[q0, packSize] = mux(guess.time,guess.state,guess.control);

%% ----------------------------------------------------------
%   COST/OBJECTIVE FUNCTION
% -----------------------------------------------------------

% Quadrature weights for trapezoid integrations
%%% TODO
% change so they are method specific
weights = ones(nGrid,1);
weights([1,end]) = 0.5;
P.objective = @(z)myCostFnc(z,packSize,OCP.pathCostFnc,OCP.bndCostFnc,weights);


%% ----------------------------------------------------------
%   CONSTRAINTS
% -----------------------------------------------------------

% NONLINEAR CONSTRAINTS
% ---------------------------
% Set direct transcription method
switch OPT.method
    case 'euler';
        defects = @euler;
    case 'euler_back';
        defects = @euler_backward;
    case 'trapezoidal'
        defects = @trapezoid;
    case 'hermiteSimpson'
        defects = @hermiteSimpsos;
    otherwise
        error('You have selected an invalid method');
end

P.nonlcon = @(z)myConstraints(z,packSize,defects,dynamics,OCP.pathCst, OCP.bndCst);

% LINEAR BOUNDARY CONSTRAINTS
% ---------------------------
% Mux lower bounds
time.lb = linspace(Bnds.initTime.lb, Bnds.finalTime.lb, nGrid);
state.lb = [Bnds.initState.lb, Bnds.state.lb*ones(1,nGrid-2), Bnds.finalState.lb];
ctrl.lb = Bnds.control.lb*ones(1,nGrid);
lb = mux(time.lb, state.lb, ctrl.lb);

% Mux upper bounds
time.ub = linspace(Bnds.initTime.ub, Bnds.finalTime.ub, nGrid);
state.ub = [Bnds.initState.ub, Bnds.state.ub*ones(1,nGrid-2), Bnds.finalState.ub];
ctrl.ub = Bnds.control.ub*ones(1,nGrid);
ub = mux(time.ub, state.ub, ctrl.ub);

%% ----------------------------------------------------------
%   FMINCON
% -----------------------------------------------------------
P.x0 = q0;
P.lb = lb;
P.ub = ub;
P.A= []; P.b = [];
P.Aeq = []; P.beq = [];
P.options = OPT.fminOpt;
P.solver = 'fmincon';

%[zSoln, objVal,exitFlag,output] = fmincon(P);
snscreen on;
snprint on;
[zSoln,fVal,exitFlag,lambda,states] = snsolve(P.objective,P.x0,P.A,P.b,P.Aeq,P.beq,P.lb,P.ub,P.nonlcon);
snprint off;
snend;

[tSoln,xSoln,uSoln] = unMux(zSoln,packSize);

%% ----------------------------------------------------------
%   SAVE RESULTS
% -----------------------------------------------------------
Soln = struct('info',[],'grid',[],'interp',[],'guess',[],'time',[]);
Soln.grid.time = tSoln;
Soln.grid.state = xSoln;
Soln.grid.control = uSoln;
% 
% Soln.interp.state = @(t)( interp1(tSoln',xSoln',t','linear',nan)' );
% fSoln = dynamics(tSoln,xSoln,uSoln);
% Soln.interp.state = @(t)( bSpline2(tSoln,xSoln,fSoln,t) );
% Soln.interp.control = @(t)( interp1(tSoln',uSoln',t','linear',nan)' );
% 
% Solver output information
% Soln.info.objVal = objVal;
% Soln.info.exitFlag = exitFlag;
% Soln.info.output = output;

end

function [ q, packSize ] = mux(t,x,u)
% MUX constructs vector to pass to NLP solver fmincon

nTime = length(t);
nState = size(x,1);
nCtrl = size(u,1);

tSpan = [t(1); t(end)];
stateCol = reshape(x, nState*nTime, 1);
ctrlCol = reshape(u, nCtrl*nTime, 1);

q = [tSpan; stateCol; ctrlCol];

packSize.nTime = nTime;
packSize.nState = nState;
packSize.nCtrl = nCtrl;
end

function [t,x,u] = unMux(z, packSize)

nTime = packSize.nTime;
nState = packSize.nState;
nCtrl = packSize.nCtrl;
nx = nState*nTime;
nu = nCtrl*nTime;

t = linspace(z(1),z(2),nTime);
x = reshape(z((3):(2+nx)),nState,nTime);
u = reshape(z((3+nx):(2+nx+nu)),nCtrl,nTime);

end

function [cost] = myCostFnc(z,packSize,pathCostFnc,bndCostFnc,weights)

% Unmux state
[t,x,u] = unMux(z, packSize);

% Compute the cost integral along trajectory
%%% TODO
% Change to running cost
if isempty(pathCostFnc)
    integralCost = 0;
else
    dt = (t(end)-t(1))/(packSize.nTime-1);
    integrand = pathCostFnc(t,x,u);  %Calculate the integrand of the cost function
    integralCost = sum(dt*integrand*weights);  %Trapazoidal integration
end

% Compute the cost at the boundaries of the trajectory
%%% TODO
% Change to Teminal cost
if isempty(bndCostFnc)
    bndCost = 0;
else
    bndCost = bndCostFnc(t,x,u);
end

cost = bndCost + integralCost;

end

function [c, ceq] = myConstraints (z,packSize,defects,dynamics,pathCst, bndCst)

% Unmux state
[t,x,u] = unMux(z, packSize);

% S.T. Dynamic's constraints
%   This is method specific

% Numerically evaluate dynamics 
dt = (t(end)-t(1))/(length(t)-1);
f = dynamics(t,x,u);
ceq_dyn = defects(dt,x,f);
ceq_dyn  = reshape(ceq_dyn,numel(ceq_dyn),1);

% S.T. Nonlinear Boundary
if isempty(bndCst);
    c_bnd = [];
    ceq_bnd = [];
else
    [c_bnd, ceq_bnd] = bndCst(t0,x0,u0,tF,xF,uF);
end

% S.T. Path
if isempty(pathCst)
    c_path = [];
    ceq_path = [];
else
    [c_path, ceq_path] = pathCst(t,x,u);
end

% Print constraints number
fprintf('\nNumber of constraints: \n')
fprintf('Dynamics: %d |',numel(ceq_dyn));
[nInvalCst, invalCst] = checkCst(ceq_dyn);
invalCst
fprintf('Path: %d |', numel(ceq_path));
[nInvalCst, invalCst] = checkCst(ceq_path);

% Costruct constraints vectors
c = [c_path; c_bnd];
ceq = [ceq_dyn; ceq_path; ceq_bnd];
end

function x = bSpline2(tGrid,xGrid,fGrid,t)
% x = bSpline2(tGrid,xGrid,fGrid,t)
%
% This function does piece-wise quadratic interpolation of a set of data.
% The quadratic interpolant is constructed such that the slope matches on
% both sides of each interval, and the function value matches on the lower
% side of the interval.
%
% INPUTS:
%   tGrid = [1, n] = time grid (knot points)
%   xGrid = [m, n] = function at each grid point in tGrid
%   fGrid = [m, n] = derivative at each grid point in tGrid
%   t = [1, k] = vector of query times (must be contained within tGrid)
%
% OUTPUTS:
%   x = [m, k] = function value at each query time
%
% NOTES:
%   If t is out of bounds, then all corresponding values for x are replaced
%   with NaN
%

[m,n] = size(xGrid);
k = length(t);
x = zeros(m, k);

% Figure out which segment each value of t should be on
[~, bin] = histc(t,[-inf,tGrid,inf]);
bin = bin - 1;

% Loop over each quadratic segment
for i=1:(n-1)
    idx = i==bin;
    if sum(idx) > 0
            h = (tGrid(i+1)-tGrid(i));
            xLow = xGrid(:,i);
            fLow = fGrid(:,i);
            fUpp = fGrid(:,i+1);
            delta = t(idx) - tGrid(i);
            x(:,idx) = bSpline2Core(h,delta,xLow,fLow,fUpp);
    end
end

% Replace any out-of-bounds queries with NaN
outOfBounds = bin==0 | bin==(n+1);
x(:,outOfBounds) = nan;

% Check for any points that are exactly on the upper grid point:
if sum(t==tGrid(end))>0
    x(:,t==tGrid(end)) = xGrid(:,end);
end

end

function x = bSpline2Core(h,delta,xLow,fLow,fUpp)
%
% This function computes the interpolant over a single interval
%
% INPUTS:
%   alpha = fraction of the way through the interval
%   xLow = function value at lower bound
%   fLow = derivative at lower bound
%   fUpp = derivative at upper bound
%
% OUTPUTS:
%   x = [m, p] = function at query times
%

%Fix dimensions for matrix operations...
col = ones(size(delta));
row = ones(size(xLow));
delta = row*delta;
xLow = xLow*col;
fLow = fLow*col;
fUpp = fUpp*col;

fDel = (0.5/h)*(fUpp-fLow);
x = delta.*(delta.*fDel + fLow) + xLow;

end

function [nInvalCst, invalCst] = checkCst(cst)

invalCst = [];
nInvalCst = 0;
for i = 1:numel(cst)
    if cst(i) ~= 0
        nInvalCst = nInvalCst + 1;
        invalCst = [invalCst, i];
    end
end

fprintf('%d/%d invalid constraints\n',nInvalCst,numel(cst));
end
