function [p1,p2,p3,p4,p5] = twoLinkLeg_Kin_wrap(t,z,params)
%INVPEND_KINEMATICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% INPUTS:
%   t:      descritize time vector
%   z:      1st order state vector [4 X n] <--> [q X n;dq X n]
%           n = number of grid/descritazation points
%   params: structure variable with model parameters
%
% OUTPUTS:
%   p1: System state derivative    
%   p2:

%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
x = z(1:2,:);
q = z(3:4,:);
dx = z(5:6,:);
dq = z(7:8,:);

[p1,p2,p3,p4,p5] = autoGen_twoLink_kin(x(1,:),x(2,:),q(1,:),q(2,:),...
                                       dx(1,:),dx(2,:),dq(1,:),dq(2,:),...
                                       p.l1,p.l2,p.d1,p.d2);
                                       
end

