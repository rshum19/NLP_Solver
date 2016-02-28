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
syms x dx ddx real
syms y dy ddy real
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
syms I2 real
syms I3 real

% Input
syms u1 real

q = [x;y;q2];
dq = [dx;dy;dq2];
z = [q;dq];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

h = sqrt(l1^2+l2^2 - 2*l1*l2*cos(q2));
q1 = asin(l2/h*sin(q2));
q3 = asin(l1/h*sin(q2));


% Link distal ends positions
P1 = [x; y];
    
P2 = P1 + [d1*sin(q1);...
           -d1*cos(q1)];
       
P3 = P1 + [l1*sin(q1);...
           -l1*cos(q1)];

P5 = [x;...
      P1(2)+h];       
       
P3d = P5 + [l2*sin(q3);...
            l2*cos(q3)];

P4 = P5 + [d2*sin(q3);...
           d2*cos(q3)];

% Center of mass position
Pcom = (m1*P1 + m2*P2 + m3*P4)./(m1 + m2 + m3);   
dPcom = subs(Pcom,{'x','y','q1'},{'x(t)','y(t)','q1(t)'});
dPcom = diff(dPcom,'t');
ddPcom = diff(dPcom,'t');
dPcom = subs(dPcom,{'x(t)','y(t)','q1(t)'},{'x','y','q1'});
ddPcom = subs(ddPcom,{'x(t)','y(t)','q1(t)'},{'x','y','q1'});

% Link distal end linear velocity
V1 = simplify(jacobian(P1,q)*dq);
V2 = simplify(jacobian(P2,q)*dq); 
V3 = simplify(jacobian(P3,q)*dq);
V4 = simplify(jacobian(P4,q)*dq);
V5 = simplify(jacobian(P5,q)*dq);


%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------

% KINETIC ENERGY KE_i = 0.5*m_i*Vc_i^2 + 0.5*I_i dq_i^2
T1 = 0.5*m1*V1.'*V1;
T1 = simplify(T1);

T2 = 0.5*m2*V2.'*V2 + 0.5*I2*dq2^2;
T2 = simplify(T2);

T3 = 0.5*m3*V4.'*V4 + 0.5*I3*dq2^2;
T3 = simplify(T3);

% Total kinetic energy
T = simplify(T1 + T2 + T3);

% POTENTIAL ENERGY PE_i = m_i*g*Yc_i;
U1 = m1*g*P1(2);
U1 = simplify(U1);

U2 = m2*g*P2(2);
U2 = simplify(U2);

U3 = m3*g*P4(2);
U3 = simplify(U3);

% Total potential energy
U = U1 + U2 + U3;
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

% Jacobian
J = jacobian(P5(2),q);

% Compute GRF
%F = (m1+m2+m3)*ddPcom(2);
%% ----------------------------------------------------------
%   GENERATE MATLAB FUNCTIONS
% -----------------------------------------------------------
% Create folder to save autogerated functions
autoFolderName = 'autogenFncs';
if ~exist(autoFolderName,'dir')
  mkdir(autoFolderName);
end

% Generate Dynamics
% -----------------
params_list = {'l1','l2',...
               'd1','d2',...
               'm1','m2','m3',...
               'I2','I3',...
               'g'};

q_list = {'x','y','q2'};
dq_list = {'dx','dy','dq2'};
ddq_list = {'ddx','ddy','ddq2'};
u_list = {'u1'};

matlabFunction(D_mtx,C_mtx, G_vec, B_mtx,J,'File',fullfile(autoFolderName,'autoGen_twoLink_dyn_mtxs.m'),...
   'vars',horzcat(q_list,dq_list,params_list));

matlabFunction(Pcom,dPcom,'File',fullfile(autoFolderName,'autoGen_CoM.m'),...
   'vars',horzcat(q_list,dq_list,params_list));

% Generate Energy
% ----------------
matlabFunction(T,U, 'File', fullfile(autoFolderName,'autoGen_twoLink_Energy.m'),...
               'vars', horzcat(q_list,dq_list,params_list),...
               'outputs',{'U','T'});

% Generate Kinematics
% --------------------
params_list = {'l1','l2',...
               'd1','d2'};

matlabFunction(P1,P2,P3,P4,P5,V1,V2,V3,V4,V5,'File',fullfile(autoFolderName,'autoGen_twoLink_kin.m'),...
                'vars',horzcat(q_list,dq_list,params_list),...
                'outputs',{'p1','p2','p3','p4','p5','dp1','dp2','dp3','dp4','dp5'});

