function [ c, ceq ] = nonlcon2( z )
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here

pack.nTime = 15;
pack.nState = 4;
pack.nControl = 1;

dyn.m1 = 1;  % elbow mass
dyn.m2 = 1; % wrist mass
dyn.g = 9.81;  % gravity
dyn.l1 = 0.5;   % length of first link
dyn.l2 = 0.5;   % length of second link

% Unpack z
[t,x,u] = unPackDecVar(z,pack);

% Compute Dynamics
dt = (t(end)-t(1))/(length(t)-1);

f = acrobotDynamics(x,u,dyn);

defects = computeDefects(dt,x,f);
pathCst = [];
bndCst = [];
[c, ceq] = collectConstraints(t,x,u,defects, pathCst, bndCst);
end

function [t,x,u] = unPackDecVar(z,pack)
%
% This function unpacks the decision variables for
% trajectory optimization into the time (t),
% state (x), and control (u) matricies
%
% INPUTS:
%   z = column vector of 2 + nTime*(nState+nControl) decision variables
%   pack = details about how to convert z back into t,x, and u
%       .nTime
%       .nState
%       .nControl
%
% OUTPUTS:
%   t = [1, nTime] = time vector (grid points)
%   x = [nState, nTime] = state vector at each grid point
%   u = [nControl, nTime] = control vector at each grid point
%

nTime = pack.nTime;
nState = pack.nState;
nControl = pack.nControl;
nx = nState*nTime;
nu = nControl*nTime;

t = linspace(z(1),z(2),nTime);

x = reshape(z((2+1):(2+nx)),nState,nTime);
u = reshape(z((2+nx+1):(2+nx+nu)),nControl,nTime);

end

function [defects, defectsGrad] = computeDefects(dt,x,f,dtGrad,xGrad,fGrad)
%
% This function computes the defects that are used to enforce the
% continuous dynamics of the system along the trajectory.
%
% INPUTS:
%   dt = time step (scalar)
%   x = [nState, nTime] = state at each grid-point along the trajectory
%   f = [nState, nTime] = dynamics of the state along the trajectory
%   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%   dtGrad = [2,1] = gradient of time step with respect to [t0; tF]
%   xGrad = [nState,nTime,nDecVar] = gradient of trajectory wrt dec vars
%   fGrad = [nState,nTime,nDecVar] = gradient of dynamics wrt dec vars
%
% OUTPUTS:
%   defects = [nState, nTime-1] = error in dynamics along the trajectory
%   defectsGrad = [nState, nTime-1, nDecVars] = gradient of defects
%


nTime = size(x,2);

idxLow = 1:(nTime-1);
idxUpp = 2:nTime;

xLow = x(:,idxLow);
xUpp = x(:,idxUpp);

fLow = f(:,idxLow);
fUpp = f(:,idxUpp);

% This is the key line:  (Trapazoid Rule)
defects = xUpp-xLow - 0.5*dt*(fLow+fUpp);

%%%% Gradient Calculations:
if nargout == 2
    
    xLowGrad = xGrad(:,idxLow,:);
    xUppGrad = xGrad(:,idxUpp,:);
    
    fLowGrad = fGrad(:,idxLow,:);
    fUppGrad = fGrad(:,idxUpp,:);
    
    % Gradient of the defects:  (chain rule!)
    dtGradTerm = zeros(size(xUppGrad));
    dtGradTerm(:,:,1) = -0.5*dtGrad(1)*(fLow+fUpp);
    dtGradTerm(:,:,2) = -0.5*dtGrad(2)*(fLow+fUpp);
    defectsGrad = xUppGrad - xLowGrad + dtGradTerm + ...
        - 0.5*dt*(fLowGrad+fUppGrad);
    
end

end
