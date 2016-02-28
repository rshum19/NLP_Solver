function drawAcrobot(t,z,p)

clf; hold on;

length = p.l1+p.l2;
axis equal; axis(length*[-1,1,-1,1]); axis off;

% Perform forward kinematics
[p1,p2] = acrobotKinematics(z,p);
pos = [[0;0],p1,p2];

% Colors:
colorGround = [118,62,12]/255;

% Plot ground:
xGrnd = length*[-1.2,1.2];
yGrnd = [0,0];
plot(xGrnd,yGrnd,'LineWidth',6,'Color',colorGround);

plot(pos(1,1:2),pos(2,1:2),'Color',[0.1, 0.8, 0.1],'LineWidth',4)
plot(pos(1,2:3),pos(2,2:3),'Color','r','LineWidth',4)
plot(pos(1,:),pos(2,:),'k.','MarkerSize',50)

title(sprintf('Acrobot Animation,  t = %6.4f', t));

drawnow; pause(0.001); 

end