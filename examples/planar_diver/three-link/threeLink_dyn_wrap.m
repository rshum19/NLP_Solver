function [ dz ] = threeLink_dyn_wrap(t,z,u,params)
%threeLink_dyn_wrap
%
%   z = [x, y, theta, alpha1, alpha2]'
%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
%% ---------------------
dz = zeros(size(z,1),nt);
for i=1:nt
    dz(:,i) = threeLink_dyn(t(i),z(:,i),u(:,i),params);
end

end

