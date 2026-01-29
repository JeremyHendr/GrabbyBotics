clear; clc; close all;
% ------ Simulation ------
Ts = 1e-4;
t = 0:Ts:10;
N = length(t);
rpm_to_rad = (2*pi)/60;
rad_to_rpm = 60/(2*pi);


R  = 2.14;           % [Ohm]
Kt = 0.0038;         % [N·m/A]
Ke = 0.011;          % [V/(rad/s)] (output shaft)

% ------ Robot ------
gear_ratio = 98.78;
D = 0.1;
r = D/2;
L = 0.20; % [m] wheel separation
m = 5; % [kg] mass of the robot
J = 0.1454; % [kg.m^2] moment of inertia of the robot (33*49 and 5kg)
b = 1.0; % friction coefficient wheels


% ========== Initialize State Matrices ==========
% ------ Robot ------
W_wheels = zeros(2,N);
T_wheels = zeros(2,N);
V = ones(2,N).*12;
V(1, t >= 2 & t <= 4) = 8; % turning phase

X = zeros(3,N);  % [v; yaw; yaw_dot;]

G = [1/(m*r)   1/(m*r);
     0         0;
     L/(2*J*r) -L/(2*J*r)]; % OK

F = [-(2*b)/(m*r)  0         0;
           0       0         1;
           0       0  -b*L^2/(2*J*r)];

inv_kynematics = [1/r 0 L/(2*r);
                  1/r 0 -L/(2*r)];

phi = eye(3) + F*Ts;
Gd  = G.*Ts;

% ------ Motor ------
T_motors = zeros(2,N);
W_motors = zeros(2,N);
current = zeros(2,N);


W_wheels(:,1) = inv_kynematics*X(:,1);


W_motors(:,1) = W_wheels(:,1)*gear_ratio;
current(:,1) = (V(:,1) - Ke*W_motors(1))/R;
T_motors(:,1) = Kt*current(1);

X(:,1) = phi*X(:,1) + Gd*T_wheels(:,1);

% ------ True state ------
X_true = zeros(4,N);
X_true(1,1) = (W_wheels(1,1)+W_wheels(2,1))*(r/2);
X_true(3,1) = (W_wheels(1,1)-W_wheels(2,1))*(r/L);
X_true(2,1) = X_true(2,1)+X_true(3,1)*Ts;
X_true(4,1) = X_true(4,1)+X_true(3,1)*Ts;

for k = 2:N
    W_wheels(:,k) = inv_kynematics*X(:,k-1);

    W_motors(:,k) = W_wheels(:,k) .* gear_ratio;
    current(:,k) = (V(:,k) - W_motors(:,k).*Ke) ./ R;
    T_motors(:,k) = current(:,k).*Kt;
    T_wheels(:,k) = T_motors(:,k).*gear_ratio;

    X(:,k) = phi*X(:,k-1) + Gd*T_wheels(:,k);
    

    % --- Perform true calculation of the state---
    X_true(1,k) = (W_wheels(1,k)+W_wheels(2,k))*(r/2); % True speed
    X_true(3,k) = (W_wheels(1,k)-W_wheels(2,k))*(r/L); % True angular speed
    X_true(2,k) = X_true(2,k-1)+X_true(3,k)*Ts;      % True angle
    X_true(4,k) = (X_true(3,k)-X_true(3,k-1))/Ts;    % True angular acceleration
end

% ========== Plotting ==========
% ------------------ Motor ------------------
figure;
% Speed plot
subplot(3,1,1);
% yyaxis left
plot(t, W_wheels(1,:).*rad_to_rpm, 'b'); hold on;
plot(t, W_wheels(2,:).*rad_to_rpm, 'r'); hold on;
ylim([0 110]);
% yyaxis right
% plot(t, W_motors(1,:).*rad_to_rpm, 'g'); hold on;
% plot(t, W_motors(2,:).*rad_to_rpm, 'y'); hold on;
% ylim([0 11000]);
title('Wheel speed');
xlabel('Time [s]');
ylabel('speed [RPM]');
grid on;

% Torque plot
subplot(3,1,2);
plot(t, T_wheels(1,:), 'r', 'LineWidth', 2); hold on;
plot(t, T_wheels(2,:), 'b', 'LineWidth', 2); hold on;
title('Wheel Output torque');
xlabel('Time [s]');
ylabel('Torque [N.m]');
grid on;


% Current plot
subplot(3,1,3);
plot(t, current(1,:), 'r', 'LineWidth', 2); hold on;
plot(t, current(2,:), 'b', 'LineWidth', 2); hold on;
title('Motor current');
xlabel('Time [s]');
ylabel('Ampere [A]');
grid on;

% ------------------ Robot ------------------
figure;
% Speed plot
subplot(3,1,1);
plot(t, X(1,:), 'r'); hold on;
plot(t, X_true(1,:), 'b'); 
title('Robot speed');
xlabel('Time [s]');
ylabel('speed [m/s]');
grid on; legend('Estimate','Truth');

% Angular speed
subplot(3,1,2);
plot(t, X(2,:), 'r', 'LineWidth', 2); hold on;
plot(t, X_true(2,:), 'b', 'LineWidth', 2);
title('Robot angle');
xlabel('Time [s]');
ylabel('angle [rad]');
grid on; legend('Estimate','Truth');


% Robot angular acceleration
subplot(3,1,3);
plot(t, X(3,:), 'r', 'LineWidth', 2); hold on;
plot(t, X_true(3,:), 'b', 'LineWidth', 2);
title('Robot angular speed');
xlabel('Time [s]');
ylabel('angular speed [rad/s]');
grid on; legend('Estimate','Truth');