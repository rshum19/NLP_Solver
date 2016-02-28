function drawPlanarDiver(t,z,params)
%DRAWPLANARDIVER draws kinematically a 3-link planar diver
% FILENAME: drawPlanarDiver.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%

% Initialize plot
clf; hold on;

length = params.l1 + params.l2 + params.l3 ;
axis equal; axis(length*[-1,1,-1,1]); axis off;

% Perform forward kinematics
[p1,p2,p3,p4,p5] = planarDiver_KinematicsWrapper(t,z,params);
pos = [p3,p2,p1,p4,p5];

% Colors:
colorGround = [118,62,12]/255;

% Plot ground:
%xGrnd = length*[-1.2,1.2];
%yGrnd = [0,0];
%plot(xGrnd,yGrnd,'LineWidth',6,'Color',colorGround);

% Plot links:
plot(pos(1,1:2),pos(2,1:2),'Color',[0.1, 0.8, 0.1],'LineWidth',4)
plot(pos(1,2:3),pos(2,2:3),'Color','r','LineWidth',4)
plot(pos(1,3:4),pos(2,3:4),'Color','r','LineWidth',4)
plot(pos(1,4:5),pos(2,4:5),'Color','b','LineWidth',4)

% Plot joints:
plot(pos(1,2),pos(2,2),'b.','MarkerSize',50)
plot(pos(1,3),pos(2,3),'k.','MarkerSize',50)
plot(pos(1,4),pos(2,4),'k.','MarkerSize',50)

title(sprintf('Planar Diver Animation,  t = %6.4f', t));

drawnow; pause(0.001)

end

