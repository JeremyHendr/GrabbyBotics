%% Differential Drive Robot Trajectory Simulation
clear; close all; clc;

% --- Robot parameters ---
L = 0.20;        % wheel separation [m]
Ts = 0.01;       % time step [s]
T = 10;          % total simulation time [s]
t = 0:Ts:T;
wheel_diameter = 0.05; % [m]
global_speed = 1; %[m/s]
encoder_noise_deviation = 0.3; %[rev/sec]

% --- Wheel speeds (m/s) ---
% Example: robot moves forward, then turns right
rev_per_sec = global_speed/(pi*wheel_diameter);                  % [revolution/sec]
wL = rev_per_sec * ones(size(t));   % left wheel
wR = rev_per_sec * ones(size(t));   % right wheel

% Add a turning phase between 4s and 6s
wR(t > 4 & t < 8) = 6;   % slow down right wheel

% --- Generate encoder data ---
encoder_left = wL + randn(size(t)).*encoder_noise_deviation;
encoder_right = wR + randn(size(t)).*encoder_noise_deviation;

% --- Stored Posistions ---
true_pos_x = zeros(length(t),1);
true_pos_y = zeros(length(t),1);
true_pos_yaw = zeros(length(t),1);    % facing along +x axis

estimation_pos_x = zeros(length(t),1);
estimation_pos_y = zeros(length(t),1);
estimation_pos_yaw = zeros(length(t),1);    % facing along +x axis


% --- Simulation loop ---
for k = 2:length(t)
    % --- True model update ---
    true_speed = ((wR(k) + wL(k))*pi*wheel_diameter) / 2;           % linear velocity
    true_speed_yaw = ((wR(k) - wL(k))*pi*wheel_diameter) / L;       % angular velocity
    
    true_pos_x(k) = true_pos_x(k-1) + true_speed * cos(true_pos_yaw(k-1)) * Ts;
    true_pos_y(k) = true_pos_y(k-1) + true_speed * sin(true_pos_yaw(k-1)) * Ts;
    true_pos_yaw(k) = true_pos_yaw(k-1) + true_speed_yaw * Ts;

    % --- position estimation with encode data not filtered ---
    estimation_speed = ((encoder_right(k) + encoder_left(k))*pi*wheel_diameter) / 2;           % linear velocity
    estimation_speed_yaw = ((encoder_right(k) - encoder_left(k))*pi*wheel_diameter) / L;       % angular velocity
    
    estimation_pos_x(k) = estimation_pos_x(k-1) + estimation_speed * cos(estimation_pos_yaw(k-1)) * Ts;
    estimation_pos_y(k) = true_pos_y(k-1) + estimation_speed * sin(estimation_pos_yaw(k-1)) * Ts;
    estimation_pos_yaw(k) = estimation_pos_yaw(k-1) + estimation_speed_yaw * Ts;
end

% --- Plot Wheel speeds ---
figure;
subplot(2,1,1); hold on; grid on;
plot(t, wR, 'g', 'DisplayName', 'true wheel speed');
plot(t, encoder_right, 'b', 'DisplayName', 'encoder data');
ylim([3 7]);
title('Right wheel speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend;

subplot(2,1,2); hold on; grid on;
plot(t, wL, 'g', 'DisplayName', 'true wheel speed');
plot(t, encoder_left, 'b', 'DisplayName', 'encoder data');
ylim([3 7]);
title('Right wheel speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend;


% --- Plot robot trajectory ---
figure; hold on; grid on; legend;
plot(true_pos_x, true_pos_y, 'g', 'LineWidth', 2, 'DisplayName', 'True position');
plot(estimation_pos_x, estimation_pos_y, 'b', 'LineWidth', 2, 'DisplayName', 'estimation using encode data'); 
% quiver(X(1:100:end), Y(1:100:end), cos(TH(1:100:end)), sin(TH(1:100:end)), 0.2, 'r');
xlabel('X [m]'); ylabel('Y [m]');
% ylim([-2 2]); xlim([-2 20]); 
title('Differential Drive Robot Trajectory');
axis equal; grid on;


