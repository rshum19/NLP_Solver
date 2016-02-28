function [ cost ] = planarDiver_bndCostFnc(t,z,u)
%planarDiver_bndCostFnc computes the terminal cost
%
%   The terminal cost is simply how close we are to our final desired
%   orientation

%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
g = z(1:3,:);
r = z(4:5,:);
tF = t(end);

theta_tF = g(3,end);
theta_tF_des = 3*pi + pi/2;
w = 10;

cost = w.*(theta_tF - theta_tF_des).^2;

end

