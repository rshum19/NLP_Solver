%%% Dynamics for the 3-link Diver
%% xb, yb, theta are the positions of the body frame fixed to the center-of-mass of the torso.
%% alpha1 is the relative angle between the torso and hand 
%% alpha3 is the relative angle between the torso and leg

%% alpha = al ; theta = th ; tr = torso ; hd = hand ; lg = leg

%%%%%% Adding Custom Library
% addpath('\\andrew.ad.cmu.edu\users\users5\asiravur\CMU\geometry of locomotion\ProjectNew2\toolbox');
%addpath('C:\Users\Avinash\Dropbox\CMU_lap\coursework\Fall15\Geometry of Locomotion\Project Literature Survey\Three Link System\Final Code\toolbox');

syms x y th dx dy dth real                      % body frame positions and velocities
syms th1 th2 th3 real                           % absolute angles 
syms dth1 dth2 dth3 real                        % absolute velocities
syms a1 a3 real                                 % joint angles
syms da1 da3 real                               % joint velocities
syms m1 m2 m3 g real                            % masses and g
syms I1 I2 I3 real                              % moments of inertia about z-axis
syms l1 l2 l3 real                              % link lengths
syms lc1 lc2 lc3 real                           % com position
syms gen_mom_x gen_mom_y gen_mom_th                                          % generalized momentum

%% Homogenous coordinates
q = [x;y;th2;a1;a3];
a = [a1;a3];
dq = [dx;dy;dth2;da1;da3];
da = [da1;da3];
Q = [q;dq];
e2 = [0;1]; % y-axis
e1 = [1;0]; % x-axis
gen_mom = [gen_mom_x;gen_mom_y;gen_mom_th];

%% Body Positions
p2 = [x;y];                                                                        %% body-frame position
p1 = p2 + R(th2)*[0;lc2] + R(th2+a1)*[0;(l1-lc1)] ;                               %% hand CoM position
p3 = p2 - R(th2)*[0;(l2-lc2)] - R(th2-a3)*[0;lc3];  %% leg CoM position

 %% Body Velocities
 dp1 = jacobian(p1,q)*dq;
 dp2 = jacobian(p2,q)*dq;
 dp3 = jacobian(p3,q)*dq;
 
 %% Energies
%% I1*(dth2-da1)^2 + I2*dth2^2 + I3*(dth2+da3)^2  
Imat = [0 0 0 0 0;
        0 0 0 0 0;
        0 0 (I1+I2+I3) 0 0;
        0 0 0 -I1 0;
        0 0 0 0 I3];
 
 
GenMassMatrix = (Imat + jacobian(p1,q)'*m1*jacobian(p1,q) + jacobian(p2,q)'*m2*jacobian(p2,q) + jacobian(p3,q)'*m3*jacobian(p3,q));
GenMassMatrix = subs(GenMassMatrix,th2,0);

%%Locked Inertia Tensor
Ir = GenMassMatrix(1:3,1:3);
A = -Ir\GenMassMatrix(1:3,4:5);
drift = Ir\gen_mom;

Ath = A(3,:);
drift_th = drift(3);
Ath_Dot = jacobian(A(3,:),a);

%% Reduced Dynamics Mass Matrix
Js = GenMassMatrix(4:5,4:5);

 
KE = dq'*GenMassMatrix*dq;
PE = m1*g*p1(2) + m2*g*p2(2) + m3*g*p3(2);
[D,C,G] = EoM(KE,PE,q,dq);
C = C ;
G = G ; %% Coriolis and Gravity terms lumped

%% Reduced Coriolis Mass Matrix 
Fs = C(4:5,1:3)*[dx;dy;dth2] + C(4:5,4:5)*da ;
Fs = subs(Fs,{dx,dy,dth2},{0,0,A(3,:)*[da1;da3]});


matlabFunction(Ir,'file','Ir_fun');
matlabFunction(A,'file','A_fun');
matlabFunction(Ath_Dot,'file','Ath_Dot_fun');
matlabFunction(drift_th,'file','Drift_theta_fun');
matlabFunction(A(3,:),'file','Ath_fun');
matlabFunction(Js,'file','Js_fun');
matlabFunction(Fs,'file','Fs_fun');
matlabFunction(D,'file','D_fun');
matlabFunction(C,'file','C_fun');
matlabFunction(G,'file','G_fun');