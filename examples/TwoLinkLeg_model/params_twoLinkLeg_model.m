function [params] = params_twoLinkLeg_model
% params parametrs

%
%
% Parameters based on robot described in:
% [1] 
%
%---- ENVIRONMENT
g = 9.81;       % Gravity [m/s^2]

params.g = g;

%---- MASS
m1 = 30;
m2 = 3;
m3 = 5;

params.m1 = m1;  % [kg]
params.m2 = m2;  % [kg]
params.m3 = m3;  % [kg]

%---- GEOMETRY
% Link length
l1 = 1;
l2 = 1;
d1 = 0.5;
d2 = 0.5;
params.l1 = l1;    % [m]
params.l2 = l2;    % [m]
params.d1 = d1;    % [m]
params.d2 = d2;    % [m]

%---- MOMENT OF INERTIA
params.I2 = (m3*l2.^2)/12;
params.I3 = (m2*l1.^2)/12;


end