  function [ c_comp, ceq_comp] = pathCst(t,x,u,params)

% Read input
%------------------%
nGrid = size(x,2);
xL = x(1:3,:);
vL = x(4:6,:) ;
q = x(7:9,:) ; 
omega = x(10:12,:) ;

dq = zeros(3,nGrid);
R = zeros(3,3,nGrid);
for i = 1:nGrid
    dq(:,i) = vec_cross(omega(:,i), q(:,i));
    R(:,:,i) = reshape(x(13:21,i),3,3) ;
end
Omega = x(22:24,:);
%l = x(25,:);
%dl = x(26,:);

%% INEQUALITY CONSTRAINTS


%% EQUALITY CONSTRAINTS
ceq_qnorm = zeros(nGrid,1);
ceq_omeganorm = zeros(nGrid,1);
ceq_R = zeros(6,nGrid);
for i = 1:nGrid
    ceq_qnorm(i) = norm(q(:,i)) - 1;
    
    ceq_omeganorm(i) = dot(q(:,i),omega(:,i));
    
    ceq_R(:,i) = [norm(R(:,1,i))-1; norm(R(:,2,i))-1;norm(R(:,3,i))-1;...
                  dot(R(:,1,i),R(:,2,i)); dot(R(:,2,i),R(:,3,i)); dot(R(:,1,i),R(:,3,i))];
end
ceq_R = reshape(ceq_R, numel(ceq_R),1);

% Concatenate constraints
c_comp = [];
ceq_comp = [ceq_qnorm; ceq_omeganorm; ceq_R];

end

