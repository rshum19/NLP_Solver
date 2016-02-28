function plot_results(t,z,u,figHandle)

figure(figHandle)
subplot(4,1,1)
plot(t,z(1,:));
legend('\theta, orient')
xlabel('Time [sec]');
ylabel('Angle [rad]');
title('Body orientation')

subplot(4,1,2)
plot(t,z(2:3,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angle [rad]');
title('Joint angle position')

subplot(4,1,3)
plot(t,z(4:5,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angular Velocites [rad/s]');

subplot(4,1,4)
plot(t,u(1:2,:))
xlabel('Time [sec]');
ylabel('U control input');
legend('joint 1','joint 2')


end

