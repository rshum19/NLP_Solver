function [ pathCost ] = invPend_costFnc(t,x,u,params)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

nTime = length(t);
nState = size(x,1);
nControl = size(u,1);

%u = reshape(u,nControl*nTime,1);

torqueCost = sum(u.^2);  %Calculate the integrand of the cost function

pathCost = torqueCost;

end

