clear; clc; close all;

% ===== Motor parameters (Pololu 3219 output shaft) =====
R  = 2.14;           % [Ohm]
Kt = 0.3786;         % [N·m/A]
Ke = 0.011;          % [V/(rad/s)] (output shaft)
J  = 1e-5;           % [kg·m²] (approximate)
b  = 1.2e-5;         % [N·m·s/rad] (approximate)
gear_ratio = 98.78;  % reduction gearbox

% ===== Simulation setup =====
V = 12;              % [V] applied voltage step
dt = 1e-4;           % [s] time step
Tend = 0.5;          % [s] simulation time
t = 0:dt:Tend;
N = length(t);

% ===== Initialize state matrices =====
omega_motor = zeros(2,N);    % motor shaft speeds [rad/s], row1 = right, row2 = left
torque_motor = zeros(2,N);   % motor shaft torques [N·m], same order

% ===== Voltage input matrix =====
V_matrix = V * ones(2, N); % same voltage for both motors
V_matrix(1, t >= 0.2 & t <= 0.4) = 8;


% ===== Simulation loop =====
for k = 1:N-1
    % Motor currents
    i = (V_matrix(:,k) - Ke*omega_motor(:,k)) / R;   % 2x1 vector
    
    % Motor torque
    torque_motor(:,k) = Kt * i;                      % 2x1 vector
    
    % Angular acceleration
    domega = (torque_motor(:,k) - b*omega_motor(:,k)) / J;  % 2x1 vector
    
    % Integrate to get speed
    omega_motor(:,k+1) = omega_motor(:,k) + domega*dt;
end

% Final torque at steady state
torque_motor(:,end) = Kt * (V_matrix(:,end) - Ke*omega_motor(:,end)) / R;

% ===== Apply gearbox to get output shaft speed (divide by gear ratio) =====
omega_out = omega_motor / gear_ratio;   % [rad/s] output shaft
torque_out = torque_motor;              % torque already at output shaft

% ===== Plot results =====
figure;

% Torque plot
subplot(2,1,1);
plot(t, torque_out(1,:), 'b', 'LineWidth', 2); hold on;
plot(t, torque_out(2,:), 'r', 'LineWidth', 2);
title('Motor Output Torque Response');
xlabel('Time [s]');
ylabel('Torque [N·m]');
grid on;
legend('Right motor','Left motor');

% Speed plot (RPM)
subplot(2,1,2);
plot(t, omega_out(1,:)*60/(2*pi), 'b', 'LineWidth', 2); hold on;
plot(t, omega_out(2,:)*60/(2*pi), 'r', 'LineWidth', 2);
title('Motor Output Angular Speed');
xlabel('Time [s]');
ylabel('Speed [RPM]');
grid on;
legend('Right motor','Left motor');

% ===== Display steady-state values =====
Tr = torque_out(1,:);
Tl = torque_out(2,:);
fprintf('Right motor initial torque = %.2f N·m\n', Tr(1));
fprintf('Left motor initial torque = %.2f N·m\n', Tl(1));
fprintf('Right motor steady-state torque = %.2f N·m\n', Tr(end));
fprintf('Left motor steady-state torque = %.2f N·m\n', Tl(end));
fprintf('Right motor steady-state speed = %.2f RPM\n', omega_out(1,end)*60/(2*pi));
fprintf('Left motor steady-state speed = %.2f RPM\n', omega_out(2,end)*60/(2*pi));
