function [p1,p2,p3,p4,p5] = planarDiver_KinematicsWrapper(t,z,params)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

nt = length(t);
p = params;

% Extract angles
a1 = z(1,:);
a2 = z(2,:);
th = z(3,:);

% Build positions
p1 = z(1:2,:);

p4 = p1 + [p.l1/2*cos(th); p.l1/2*sin(th)];
p5 = p4 + [p.l2*cos(a2); p.l2*sin(a2)];
p2 = p1 - [p.l1/2*cos(th); p.l1/2*sin(th)];
p3 = p2 + [-p.l3*cos(a1); p.l3*sin(a1)];

end

