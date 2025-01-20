% MATLAB Code for PID Control of Reaction Wheels for a 6U CubeSat

% Clear previous data
clc; clear; close all;

% System parameters for the reaction wheel specific to a 6U CubeSat
I = 0.3;                   % Moment of inertia (kg*m^2) for a reaction wheel
K = 0.1;                   % Gain of the system (Nm/rad/s)

% Create transfer function for the reaction wheel
num = [K];                 % Numerator of the transfer function
den = [I, 0];              % Denominator of the transfer function (I*s)
sys = tf(num, den);

% PID Controller parameters - increased values for better performance
Kp = 30;                   % Proportional gain
Ki = 10;                   % Integral gain
Kd = 1;                    % Derivative gain

% Create PID controller
C = pid(Kp, Ki, Kd);

% Open-loop transfer function
sys_open_loop = series(C, sys);

% Closed-loop transfer function with feedback
sys_closed_loop = feedback(sys_open_loop, 1);

% Time vector for simulation
t = 0:0.01:10;  % Simulation time from 0 to 10 seconds

% Step response of the closed-loop system
[y, t] = step(sys_closed_loop, t);

% Plotting the results
figure;

% Plot step response
plot(t, y, 'b', 'LineWidth', 2);
title('Step Response of 6U CubeSat Reaction Wheel PID Controlled System', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Angular Velocity (rad/s)', 'FontSize', 12);
grid on;

% Adding a horizontal line at the setpoint (1.5 rad/s)
hold on;
yline(1, 'r--', 'Setpoint', 'LabelOrientation', 'horizontal', 'LineWidth', 1.5);
hold off;

% Adding legend
legend('Angular Velocity Response', 'Setpoint', 'Location', 'Best');

% Save the figure
saveas(gcf, 'reaction_wheel_pid_control_6U_setpoint_1_5_high_gains.png');
