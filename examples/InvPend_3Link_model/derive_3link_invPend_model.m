% FILENAME: derive_3link_invPend_model.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 1/23/2016
%
% DESCRIPTION:
% This script will derive the equation of motion using the 
% Lagrian Dynamics method. Refer to Apendix B.4.5 in [1] 
% for derivation.
% The model is written in the form:
%       D(q)ddq + C(q,dq)dq + G(q) = B(q)u
% 
% Notation: 
% 
% References:
% [1] Westervelt, Eric R., et al. Feedback control of dynamic 
%     bipedal robot locomotion. Vol. 28. CRC press, 2007.
%
% 
%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clc; clear; close all;

%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------

% Environment 
syms g real         % gravity

%  Absolute joint angles and velocities
syms q1 dq1 ddq1 real
syms q2 dq2 ddq2 real
syms q3 dq3 ddq3 real 

% Individual link length
syms l1 d1 real
syms l2 d2 real
syms l3 d3 real 

% Individual link masses
syms m1 m2 m3 real

% Links inertias
syms I1 real
syms I2 real
syms I3 real

% Foot position
syms xb real
syms yb real

% Input
syms u1 real
syms u2 real
syms u3 real

% Relative angles
syms q1L real   % q1L = q1
syms q2L real   % q2L = q1-q2
syms q3L real   % q3L = q2-q3

q = [q1; q2; q3];
qL = [q1L; q2L; q3L];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];
u = [u2; u3];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% Link distal ends positions
P1 = [l1*sin(q1);...
       l1*cos(q1)];
P2 = [l1*sin(q1) + l2*sin(q2);...
       l1*cos(q1) + l2*cos(q2)];
P3 = [l1*sin(q1) + l2*sin(q2) + l3*sin(q3);...
       l1*cos(q1) + l2*cos(q2) + l3*cos(q3)];

% Link center of mass positions
Pc1 = [d1*sin(q1);...
       d1*cos(q1)];
Pc2 = [l1*sin(q1) + d2*sin(q2);...
       l1*cos(q1) + d2*cos(q2)];
Pc3 = [l1*sin(q1) + l2*sin(q2) + d3*sin(q3);...
       l1*cos(q1) + l2*cos(q2) + d3*cos(q3)];
   
% Center of mass position
Pcom = (m1*Pc1 + m2*Pc2 + m3*Pc3)./(m1 + m2 + m3);

%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------

% Link distal end linear velocity
V1 = jacobian(P1,q)*dq;
V2 = jacobian(P2,q)*dq;
V3 = jacobian(P3,q)*dq;

% Link center of mass linear velocity
Vc1 = jacobian(Pc1,q)*dq;
Vc2 = jacobian(Pc2,q)*dq;
Vc3 = jacobian(Pc3,q)*dq;

% KINETIC ENERGY KE_i = 0.5*m_i*Vc_i^2 + 0.5*I_i dq_i^2
KE1 = 0.5*m1*Vc1.'*Vc1 + 0.5*I1*dq1^2;
KE1 = simplify(KE1);
KE2 = 0.5*m2*Vc2.'*Vc2 + 0.5*I2*dq2^2;
KE2 = simplify(KE2);
KE3 = 0.5*m3*Vc3.'*Vc3 + 0.5*I3*dq3^2;
KE3 = simplify(KE3);

% Total kinetic energy
KE = KE1 + KE2 + KE3;
KE = simplify(KE);


% POTENTIAL ENERGY PE_i = m_i*g*Yc_i;
PE1 = m1*g*Pc1(2);
PE1 = simplify(PE1);
PE2 = m2*g*Pc2(2);
PE2 = simplify(PE2);
PE3 = m3*g*Pc3(2);
PE3 = simplify(PE3);

% Total potential energy
PE = PE1 + PE2 + PE3;
PE = simplify(PE);

% COMPUTING EOM MATRICIES

% Gravity vector
G_vec = simplify(jacobian(PE,q).');

% Mass-Inertia matrix
D_mtx = simplify(jacobian(KE,dq).');
D_mtx = simplify(jacobian(D_mtx,dq));

% Coriolis and centrifugal matrix
syms C_mtx real
n=max(size(q));
for k=1:n
    for j=1:n
        C_mtx(k,j) = 0*g;
        for i=1:n
            C_mtx(k,j) = C_mtx(k,j)+ 1/2*(diff(D_mtx(k,j),q(i)) + ...
                diff(D_mtx(k,i),q(j)) - ...
                diff(D_mtx(i,j),q(k)))*dq(i);
        end
    end
end
C_mtx = simplify(C_mtx);

% Input matrix  
 Gamma_0 = [q1; q1-q2; q2-q3];
B_mtx = simplify(jacobian(Gamma_0,q));
B_mtx = B_mtx'*[zeros(1,2);eye(2,2)];

%% ----------------------------------------------------------
%   GENERATE MATLAB FUNCTIONS
% -----------------------------------------------------------
% Generate Dynamics
% ----------------
params_list = {'l1','l2','l3',...
               'd1','d2','d3',...
               'm1','m2','m3',...
               'I1','I2','I3',...
               'g'};

q_list = {'q1','q2','q3'};
dq_list = {'dq1','dq2','dq3'};
u_list = {'u2','u3'};

matlabFunction(D_mtx,C_mtx, G_vec, B_mtx,'File','invPend_3link_EOM_mtxs.m',...
   'vars',horzcat(q_list,dq_list,u_list,params_list));

% Generate Energy
% ----------------
matlabFunction(KE,PE, 'File', 'invPend_2DoF_Energy.m',...
               'vars', horzcat(q_list,dq_list,params_list),...
               'outputs',{'U','T'});

% Generate Kinematics
% --------------------
params_list = {'l1','l2','l3'};

matlabFunction(P1,P2,P3,V1,V2,V3,'File','invPend_3link_kin.m',...
                'vars',horzcat(q_list,dq_list,params_list),...
                'outputs',{'p1','p2','p3','dp1','dp2','dp3'});
            
