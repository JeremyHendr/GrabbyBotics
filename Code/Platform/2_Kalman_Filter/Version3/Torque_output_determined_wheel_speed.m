clear; clc; close all;
% ------ Simulation ------
Ts = 1e-4;
t = 0:Ts:0.5;
N = length(t);
rpm_to_rad = (2*pi)/60;
rad_to_rpm = 60/(2*pi);

% ------ Motor ------
R  = 2.14;           % [Ohm]
Kt = 0.0038;         % [N·m/A]
Ke = 0.011;          % [V/(rad/s)] (output shaft)

% ------ Rboot ------
gear_ratio = 98.78;
D = 0.1;

% ========== Initialize State Matrices ==========
% ------ Robot ------
W_wheel = ones(1,N).*rpm_to_rad*100;
W_wheel(1, t >= 0.2 & t <= 0.4) = rpm_to_rad*0;
V = ones(1,N).*12;

% ------ Motor ------
T_motor = zeros(1,N);
W_motor = zeros(1,N);
current = zeros(1,N);

W_motor(1) = W_wheel(1)*gear_ratio;
current(1) = (V(1) - Ke*W_motor(1))/R;
T_motor(1) = Kt*current(1);

for k = 2:N
    W_motor(k) = W_wheel(k) * gear_ratio;
    current(k) = (V(k) - Ke * W_motor(k)) / R;
    T_motor(k) = Kt * current(k);

end

% Speed plot
subplot(3,1,1);
yyaxis left
plot(t, W_wheel.*rad_to_rpm, 'b'); hold on;
ylim([0 110]);
yyaxis right
plot(t, W_motor.*rad_to_rpm, 'r'); hold on;
ylim([0 11000]);
title('Wheel speed');
xlabel('Time [s]');
ylabel('speed [RPM]');
grid on;

% Torque plot
subplot(3,1,2);
plot(t, T_motor.*gear_ratio, 'b', 'LineWidth', 2);
title('Wheel Output torque');
xlabel('Time [s]');
ylabel('Torque [N.m]');
grid on;


% Current plot
subplot(3,1,3);
plot(t, current, 'b', 'LineWidth', 2); hold on;
title('Motor current');
xlabel('Time [s]');
ylabel('Ampere [A]');
grid on;