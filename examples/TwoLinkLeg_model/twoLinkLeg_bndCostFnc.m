function [ cost ] = twoLinkLeg_bndCostFnc(t,z,u,params)

p = params;
nt = length(t);
x = z(1:2,:);
q = z(3:4,:);
dx = z(5:6,:);
dq = z(7:8,:);

F = zeros(1,nt);
for i = 1:nt
[Pcom,dPcom] = autoGen_CoM(x(1,i),x(2,i),q(1,i),q(2,i),...
                           dx(1,i),dx(2,i),dq(1,i),dq(2,i),...
                           p.l1,p.l2,...
                           p.d1,p.d2,...
                           p.m1,p.m2,p.m3,...
                           p.I2,p.I3,...
                           p.g);
    F(:,i) = getGRF( Pcom(2), dPcom(2));
end

cost = max(F);

end

