clc
clear

q_1 = load("hw7_q.txt");
qdot_1 = load("hw7_qdot.txt");
x_desired_1 = load("hw7_x_desired.txt");
x_1 = load("hw7_x.txt");
torque_1 = load("hw7_torque.txt");
x_rot_error_1 = load("hw7_x_rot_error.txt");
time_stamp_1 = load("hw7_time_stamp.txt");

figure(1)
subplot(1,3,1);
plot(time_stamp_1,x_1(:,1),time_stamp_1,x_desired_1(:,1))
ylim([-0.1 0.5])
legend('x','x desired','Location','north')
title('x position','FontSize',12)
xlabel('time(s)')
ylabel('position(m)')
grid on

subplot(1,3,2);
plot(time_stamp_1,x_1(:,2),time_stamp_1,x_desired_1(:,2))
ylim([-0.3 0.3])
legend('x','x desired','Location','north')
title('y position','FontSize',12)
xlabel('time(s)')
ylabel('position(m)')
grid on

subplot(1,3,3);
plot(time_stamp_1,x_1(:,3),time_stamp_1,x_desired_1(:,3))
ylim([0.3 0.9])
legend('x','x desired','Location','north')
title('z position','FontSize',12)
xlabel('time(s)')
ylabel('position(m)')
grid on

figure(2)
plot(time_stamp_1,x_rot_error_1(:,4),time_stamp_1,x_rot_error_1(:,5),time_stamp_1,x_rot_error_1(:,6))
% ylim([-0.05 0.05])
legend('x error','y error','z error', 'Location','north')
title('rotation error','FontSize',12)
xlabel('time(s)')
ylabel('theta(rad)')
grid on

% figure(9)
% plot(time_stamp_2,qdot_2(:,1),time_stamp_2,qdot_2(:,2),time_stamp_2,qdot_2(:,3),time_stamp_2,qdot_2(:,4),time_stamp_2,qdot_2(:,5),time_stamp_2,qdot_2(:,6),time_stamp_2,qdot_2(:,7))
% % ylim([0.2 0.6])
% legend('1st','2nd','3rd','4th','5th','6th','7th','Location','north')
% title('q dot','FontSize',12)
% xlabel('time(s)')
% ylabel('rad/s')
% grid on