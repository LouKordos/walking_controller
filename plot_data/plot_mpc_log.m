% Plots the MPC log CSV files from controller running on Gazebo
% Read the file
data = csvread('mpc_log.csv');
#data = csvread('/home/loukas/.gazebo/models/simplified_biped/control_plugin/mpc_log.csv');
%graphics_toolkit("qt")
% Extract the data for easier access
t       = data(:,1);
phi     = data(:,2);
phi_delay_step = data(:,29);
theta   = data(:,3);
theta_delay_step = data(:,27);
psi     = data(:,4);
pos_x   = data(:,5);
pos_y   = data(:,6);
pos_z   = data(:,7);
omega_x = data(:,8);
omega_y = data(:,9);
omega_z = data(:,10);
vel_x   = data(:,11);
vel_y   = data(:,12);
vel_z   = data(:,13);
g       = data(:,14);
f_x_left    = data(:,15);
f_y_left    = data(:,16);
f_z_left    = data(:,17);
f_x_right   = data(:,18);
f_y_right   = data(:,19);
f_z_right   = data(:,20);
r_x_left    = data(:,21);
r_y_left    = data(:,22);
r_z_left    = data(:,23);
r_x_right   = data(:,24);
r_y_right   = data(:,25);
r_z_right   = data(:,26);

full_iteration_time = data(:,28);

% Plot the CoM orientation
figure(1); clf;
plot(t, phi,'linewidth', 2); hold on; grid on;
plot(t, phi_delay_step, 'linewidth', 1, '--');
plot(t, theta,'linewidth', 2);
plot(t, theta_delay_step,'linewidth', 1, '--');
plot(t, psi, 'linewidth', 1.5);
legend('phi', 'phi_delay_step', 'theta','theta_delay_step', 'psi');
title('Angles');
xlabel('Time [s]');
ylabel('Angle [rad]');

% Plot the CoM angular velocities
figure(2); clf;
plot(t, omega_x,'linewidth', 1.5); hold on; grid on;
plot(t, omega_y,'linewidth', 1.5);
plot(t, omega_z, 'linewidth', 1.5);
legend('omega_x', 'omega_y', 'omega_z');
title('Angular velocities');
xlabel('Time [s]');
ylabel('Omega [rad/s]');

% Plot the CoM position
figure(3); clf;
plot(t, pos_x, 'linewidth', 1.5); hold on; grid on;
plot(t, pos_y, 'linewidth', 1.5);
plot(t, pos_z, 'linewidth', 1.5);
legend('x', 'y', 'z');
title('Position');
xlabel('Time [s]');
ylabel('Pos [m]');

% Plot the force positions
figure(5); clf;
subplot(2,1,1);
plot(t, r_x_left,'linewidth', 1.5); hold on; grid on;
plot(t, r_y_left,'linewidth', 1.5);
plot(t, r_z_left,'linewidth', 1.5);
legend('x', 'y', 'z');
title('Location left foot');
xlabel('Tie [s]');
ylabel('Pos [m]');
subplot(2,1,2);
plot(t, r_x_right,'linewidth', 1.5); hold on; grid on;
plot(t, r_y_right,'linewidth', 1.5);
plot(t, r_z_right,'linewidth', 1.5);
legend('x', 'y', 'z');
title('Location right foot');
xlabel('Time [s]');
ylabel('Pos [m]');

% Plot the forces
figure(6); clf;
subplot(2,1,1);
plot(t, f_x_left, 'linewidth', 1.5); hold on; grid on;
plot(t, f_y_left, 'linewidth', 1.5);
plot(t, f_z_left, 'linewidth', 1.5);
legend('x', 'y', 'z');
title('Forces left foot');
xlabel('Time [s]');
ylabel('Force [N]');
subplot(2,1,2);
plot(t, f_x_right,'linewidth', 1.5); hold on; grid on;
plot(t, f_y_right,'linewidth', 1.5);
plot(t, f_z_right,'linewidth', 1.5);
legend('x', 'y', 'z');
title('Forces right foot');
xlabel('Time [s]');
ylabel('Force [N]');

figure(7); clf;
plot(t, full_iteration_time,'linewidth', 1.5); hold on; grid on;
legend('Full iteration time');
title('Full iteration time');
xlabel('Time [s]');
ylabel('Full iteration time [ms]');

figure(420); clf;
plot(t, vel_x, 'linewidth', 1.5); hold on; grid on;
plot(t, vel_y, 'linewidth', 1.5);
plot(t, vel_z, 'linewidth', 1.5);
legend('vel_x', 'vel_y', 'vel_z');
title("Linear velocities");
xlabel('Time [s]');
ylabel('Velocity [m/s]');

%tau_1_left = data(:,27);
%tau_2_left = data(:,28);
%tau_3_left = data(:,29);
%tau_4_left = data(:,30);
%tau_5_left = data(:,31);
%
%tau_1_right = data(:,32);
%tau_2_right = data(:,33);
%tau_3_right = data(:,34);
%tau_4_right = data(:,35);
%tau_5_right = data(:,36);
%
%theta_1_left = data(:,37);
%theta_2_left = data(:,38);
%theta_3_left = data(:,39);
%theta_4_left = data(:,40);
%theta_5_left = data(:,41);
%
%theta_1_dot_left = data(:,42);
%theta_2_dot_left = data(:,43);
%theta_3_dot_left = data(:,44);
%theta_4_dot_left = data(:,45);
%theta_5_dot_left = data(:,46);
%
%theta_1_right = data(:,47);
%theta_2_right = data(:,48);
%theta_3_right = data(:,49);
%theta_4_right = data(:,50);
%theta_5_right = data(:,51);
%
%theta_1_dot_right = data(:,52);
%theta_2_dot_right = data(:,53);
%theta_3_dot_right = data(:,54);
%theta_4_dot_right = data(:,55);
%theta_5_dot_right = data(:,56);
%
%% Plot the leg torques
%figure(7); clf;
%subplot(2,1,1);
%plot(t, tau_1_left, 'linewidth', 1.5); hold on; grid on;
%plot(t, tau_2_left, 'linewidth', 1.5);
%plot(t, tau_3_left, 'linewidth', 1.5);
%plot(t, tau_4_left, 'linewidth', 1.5);
%plot(t, tau_5_left, 'linewidth', 1.5);
%legend('hip3', 'hip2', 'hip1', 'knee', 'ankle');
%title('Left leg torques');
%xlabel('Time [s]');
%ylabel('Torque [Nm]');
%ylim([-200, 200])
%subplot(2,1,2);
%plot(t, tau_1_right, 'linewidth', 1.5); hold on; grid on;
%plot(t, tau_2_right, 'linewidth', 1.5);
%plot(t, tau_3_right, 'linewidth', 1.5);
%plot(t, tau_4_right, 'linewidth', 1.5);
%plot(t, tau_5_right, 'linewidth', 1.5);
%legend('hip3', 'hip2', 'hip1', 'knee', 'ankle');
%title('Right leg torques');
%xlabel('Time [s]');
%ylabel('Torque [Nm]');
%ylim([-200, 200])
%
%% Plot the leg angles
%figure(8); clf;
%subplot(2,1,1);
%plot(t, theta_1_left, 'linewidth', 1.5); hold on; grid on;
%plot(t, theta_2_left, 'linewidth', 1.5);
%plot(t, theta_3_left, 'linewidth', 1.5);
%plot(t, theta_4_left, 'linewidth', 1.5);
%plot(t, theta_5_left, 'linewidth', 1.5);
%legend('hip3', 'hip2', 'hip1', 'knee', 'ankle');
%title('Left leg angles');
%xlabel('Time [s]');
%ylabel('Angle [rad]');
%ylim([-5, 5])
%subplot(2,1,2);
%plot(t, theta_1_right, 'linewidth', 1.5); hold on; grid on;
%plot(t, theta_2_right, 'linewidth', 1.5);
%plot(t, theta_3_right, 'linewidth', 1.5);
%plot(t, theta_4_right, 'linewidth', 1.5);
%plot(t, theta_5_right, 'linewidth', 1.5);
%legend('hip3', 'hip2', 'hip1', 'knee', 'ankle');
%title('Right leg torques');
%xlabel('Time [s]');
%ylabel('Angle [rad]');
%ylim([-5, 5])
%
%%Plot the leg angular velocities
%figure(9); clf;
%subplot(2,1,1);
%plot(t, theta_1_dot_left, 'linewidth', 1.5); hold on; grid on;
%plot(t, theta_2_dot_left, 'linewidth', 1.5);
%plot(t, theta_3_dot_left, 'linewidth', 1.5);
%plot(t, theta_4_dot_left, 'linewidth', 1.5);
%plot(t, theta_5_dot_left, 'linewidth', 1.5);
%legend('hip3', 'hip2', 'hip1', 'knee', 'ankle');
%title('Left leg angular velocities');
%xlabel('Time [s]');
%ylabel('Angular velocity [rad/s]');
%ylim([-20, 20])
%subplot(2,1,2);
%plot(t, theta_1_dot_right, 'linewidth', 1.5); hold on; grid on;
%plot(t, theta_2_dot_right, 'linewidth', 1.5);
%plot(t, theta_3_dot_right, 'linewidth', 1.5);
%plot(t, theta_4_dot_right, 'linewidth', 1.5);
%plot(t, theta_5_dot_right, 'linewidth', 1.5);
%legend('hip3', 'hip2', 'hip1', 'knee', 'ankle');
%title('Right leg angular velocities');
%xlabel('Time [s]');
%ylabel('Angular velocity [rad/s]');
%ylim([-20, 20])