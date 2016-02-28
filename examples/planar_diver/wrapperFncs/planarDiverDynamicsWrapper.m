function [ dz ] = planarDiverDynamicsWrapper(t,z,u)
%planarDiverDynamicsWrapper
%
%   z = [x, y, theta, alpha1, alpha2]'
%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
r = z(1:2,:);
dr = z(4:5,:);
g = z(1:3,:);
dg = z(4:6,:);
%% ----------------------------------------------------------
%   REDUCED EQUATION OF MOTION
% -----------------------------------------------------------

% allocate memory space
zeta = zeros(size(g));
ddr = zeros(size(r));

for i=1:nt
    
    % Joint angle velocities
    ddr(:,i) = u(:,i);
    
    % Body velocities
    alpha1 = r(1,i);
    alpha2 = r(2,i);
    
    a31 = 5 + 3*cos(alpha2) + cos(alpha1-alpha2);
    a32 = 5 + 3*cos(alpha1) + cos(alpha1-alpha2);
    D = 19 + 16*(cos(alpha1) + cos(alpha2)) + 2*cos(alpha1-alpha2);
    
    A = zeros(size(g,1),size(r,1));
    A(3,:) = [a31, a32];
    
    zeta(:,i) = -1/D*A*dr(:,i);
end

ddg = [ddr;zeta(3,:)];

dz = [dg;ddg];

end

