function [c, c_eq ] = nonlcon(z)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

p.m1 = 1;  % elbow mass
p.m2 = 1; % wrist mass
p.g = 9.81;  % gravity
p.l1 = 0.5;   % length of first link
p.l2 = 0.5;   % length of second link

nTime = 15;
nState = 4;
nControl = 1;
nx = nState*nTime;
nu = nControl*nTime;

t = linspace(z(1),z(2),nTime);
x = reshape(z((2+1):(2+nx)),nState,nTime);
u = reshape(z((2+nx+1):(2+nx+nu)),nControl,nTime);

nGrid = size(x,2);

dt = (t(end)-t(1))/(length(t)-1);

idxNow = 1:(nGrid-1);
idxNxt = 2:nGrid;

xNow = x(:,idxNow);
xNxt = x(:,idxNxt);

dynamics = @(t,x,u)(acrobotDynamics(x,u,p));

f = dynamics(t,x,u);

fNow = f(:,idxNow);
fNxt = f(:,idxNxt);

% s.t. dynamics
c_dyn = xNxt - xNow - 0.2*dt*(fNow + fNxt);


c = [];

c_eq = reshape(c_dyn,numel(c_dyn),1);

end

