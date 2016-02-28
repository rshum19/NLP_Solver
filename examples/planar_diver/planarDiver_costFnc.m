function [ cost ] = planarDiver_costFnc(t,x,u,params)

nTime = length(t);
nState = size(x,1);
nControl = size(u,1);

% Running cost
cost = u.^2;

end

