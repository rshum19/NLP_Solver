function drawInvPend3(t,z,params)
%DRAWINVPEND3 draws kinematically a 3-link inverted pendulum
% FILENAME: drawInvPend3.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%

% Initialize plot
clf; hold on;

length = params.l1 + params.l2 ;
axis equal; axis(length*[-1,1,-1,1]); axis off;

% Perform forward kinematics
[p1,p2,p3,p4,p5] = twoLinkLeg_Kin_wrap(t,z,params);
p5 = [0;p5];
pos = [p1,p3,p5];

% Colors:
colorGround = [118,62,12]/255;

% Plot ground:
xGrnd = length*[-1.2,1.2];
yGrnd = [0,0];
plot(xGrnd,yGrnd,'LineWidth',6,'Color',colorGround);

% Plot links:
plot(pos(1,1:2),pos(2,1:2),'Color',[0.1, 0.8, 0.1],'LineWidth',4)
plot(pos(1,2:3),pos(2,2:3),'Color','r','LineWidth',4)
%plot(pos(1,3:4),pos(2,3:4),'Color','b','LineWidth',4)
%plot(pos(1,4:5),pos(2,4:5),'Color','y','LineWidth',4)

% Plot joints:
%plot(pos(1,1),pos(1,1),'k.','MarkerSize',50)
%plot(pos(1,2),pos(2,2),'k.','MarkerSize',50)
%plot(pos(1,3),pos(2,3),'k.','MarkerSize',50)

title(sprintf('2Link inverted Pendulum Animation,  t = %6.4f', t));

drawnow; pause(0.001)

end

