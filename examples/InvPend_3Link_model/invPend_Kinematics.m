function [p1,p2,p3,dp1,dp2,dp3] = invPend_Kinematics(t,z,params)
%INVPEND_KINEMATICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% INPUTS:
%   t:      descritize time vector
%   z:      1st order state vector [6 X n] <--> [q X n;dq X n]
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
q1 = z(1,:);
q2 = z(2,:);
q3 = z(3,:);
dq1 = z(4,:);
dq2 = z(5,:);
dq3 = z(6,:);

[p1,p2,p3,dp1,dp2,dp3] = invPend_3link_kin(q1,q2,q3,...
                                           dq1,dq2,dq3,...
                                           p.l1,p.l2,p.l3);
                                       
end

