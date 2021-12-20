clc
clear

project_ = load("projecyt.txt");

time_1 = project_(:,1);
x_ = project_(:,2:4);
xdot_ = project_(:,5:7);

figure(1)
subplot(1,2,1);
% plot(time_1,x_(:,1),time_1,x_(:,2),time_1,x_(:,3))
plot(time_1,x_(:,3))
ylim([0.13 0.17])
% legend('x position','y position','z position','Location','north')
legend('z position','Location','north')
title('position','FontSize',12)
xlabel('time(s)')
ylabel('position(m)')
grid on

subplot(1,2,2);
plot(time_1,xdot_(:,1),time_1,xdot_(:,2),time_1,xdot_(:,3))
ylim([-0.4 0.6])
legend('x dot','y dot','z dot','Location','north')
title('velocity','FontSize',12)
xlabel('time(s)')
ylabel('velocity(m/s)')
grid on
