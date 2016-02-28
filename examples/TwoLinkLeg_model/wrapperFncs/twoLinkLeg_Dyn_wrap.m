function [DZ,F] = twoLinkLeg_Dyn_wrap(t,z,u,params)


%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
x = z(1:2,:);
q = z(3,:);
dx = z(4:5,:);
dq = z(6,:);
dz = [dx;dq];

if isempty(u)
    u = zeros(1,nt);
end
%% ----------------------------------------------------------
%   EQUATION OF MOTION
% -----------------------------------------------------------
% Allocate memory
ddz = zeros(size(dz,1),nt);
F = zeros(1,nt);
for i = 1:nt
 
    % EOM
    [D_mtx,C_mtx, G_vec,B_mtx,J] = autoGen_twoLink_dyn_mtxs(x(1,i),x(2,i),q(1,i),...
                                                       dx(1,i),dx(2,i),dq(1,i),...
                                                       p.l1,p.l2,...
                                                       p.d1,p.d2,...
                                                       p.m1,p.m2,p.m3,...
                                                       p.I2,p.I3,...
                                                       p.g);
    
    [Pcom,dPcom] = autoGen_CoM(x(1,i),x(2,i),q(1,i),...
                           dx(1,i),dx(2,i),dq(1,i),...
                           p.l1,p.l2,...
                           p.d1,p.d2,...
                           p.m1,p.m2,p.m3,...
                           p.I2,p.I3,...
                           p.g);
    
    [p1,p2,p3,p4,p5,v1,v2,v3,v4,v5] = twoLinkLeg_Kin_wrap(t,z,params);
    F(:,i) = getGRF( p5, v5);
                                                   
    % Box acceleration                                               
    ddz(:,i) = D_mtx\(B_mtx*u(:,i) + J'*F(:,i) - C_mtx*dz(:,i) - G_vec); 
    ddz(1,i) = 0;
    
end

DZ = [dz;ddz];
end

