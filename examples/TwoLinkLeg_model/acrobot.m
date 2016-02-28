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

% Individual link length
syms l1 d1 real
syms l2 d2 real

% Individual link masses
syms m1 real
syms m2 real
syms m3 real

% Links inertias
syms I1 real
syms I2 real

% Input
syms u1 real

q = [q1; q2];
dq = [dq1; dq2];
x = [q;dq];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% Link distal ends positions
P1 = [l1*sin(q1);...
      -l1*cos(q1)];
   
P2 = P1 + [l2*sin(q1+q2);...
           d2*sin(q1+q2)];

% Center of mass position
Pcom = (m1*P1 + m2*P2)./(m1 + m2);       
      
% Link distal end linear velocity
V1 = jacobian(P1,q)*dq;
V2 = jacobian(P2,q)*dq; 

%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------

% KINETIC ENERGY KE_i = 0.5*m_i*Vc_i^2 + 0.5*I_i dq_i^2
T1 = 0.5*m1*V1.'*V1 + 0.5*I1*dq1^2;
T1 = simplify(T1);

T2 = 0.5*m2*V2.'*V2 + 0.5*I2*dq2^2;
T2 = simplify(T2);

% Total kinetic energy
T = simplify(T1 + T2);

% POTENTIAL ENERGY PE_i = m_i*g*Yc_i;
U1 = m1*g*P1(2);
U1 = simplify(U1);

U2 = m2*g*P2(2);
U2 = simplify(U2);

% Total potential energy
U = U1 + U2;
U = simplify(U);

% COMPUTING EOM MATRICIES
% -----------------------
% Gravity vector
G_vec = simplify(jacobian(U,q).');

% Mass-Inertia matrix
D_mtx = simplify(jacobian(T,dq).');
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
% assumes you have knee torques
Phi_0 = q2;
B_mtx = simplify(jacobian(Phi_0,q));
B_mtx = B_mtx';   