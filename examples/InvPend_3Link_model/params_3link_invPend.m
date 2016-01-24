function [params] = params_5link_biped
% params parametrs

%
%
% Parameters based on robot described in:
% [1] Tzafestas, Spyros, Mark Raibert, and Costas Tzafestas. 
%     "Robust sliding-mode control applied to a 5-link biped robot." 
%     Journal of Intelligent and Robotic Systems 15.1 (1996): 67-133. 
%
%---- ENVIRONMENT
g = 9.81;       % Gravity [m/s^2]

params.g = g;

%---- MASS
torso_mass = 15.96;  % [kg]
thigh_mass = 2.076;  % [kg]
shank_mass = 4.166;  % [kg]

params.m1 = shank_mass;
params.m2 = thigh_mass;
params.m3 = torso_mass;


%---- MOMENT OF INERTIA
torso_inertia = 0.724;  % [kg m]
thigh_inertia = 0.239;  % [kg m]
shank_inertia = 0.321;  % [kg m]

params.I1 = shank_inertia;
params.I2 = thigh_inertia;
params.I3 = torso_inertia;

%---- GEOMETRY
% Link length
torso_length = 0.53;   % [m]
thigh_length = 0.30;   % [m]
shank_length = 0.37;   % [m]

params.l1 = shank_length;
params.l2 = thigh_length;
params.l3 = torso_length;


% COM distance from joint
torso_com = 0.31;  % [m]
thigh_com = 0.15;  % [m]
shank_com = 0.20;  % [m]

params.d1 = shank_com;
params.d2 = thigh_com;
params.d3 = torso_com;

end

