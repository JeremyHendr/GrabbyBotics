clear; clc; close all;

% --- VARIABLE INITIALIZATION ---
Ts = 0.1; %sampling period
T = 10; %Simulation duration
t = 0:Ts:T;

r = 0.05; % [m] wheel radius
L = 0.20; % [m] wheel separation
m = 5; % [kg] mass of the robot
J = 0.1454; % [kg.m^2] moment of inertia of the robot (33*49 and 5kg)
b = 1.0; % friction coefficient wheels


% ------ Wheel  ------
Tr = ones(size(t)).*1; % [N.m] Torque on right wheel in Nm (stall torque is 2.11
Tl = ones(size(t)).*1;

% Add a turning phase between 4s and 6s
% Tr(t > 4 & t < 8) = 0.5;   % slow down right wheel

G = [1/(m*r)    1/(m*r);
     L/(2*J*r) -L/(2*J*r)];

F = [-(2*b)/(m*r) 0;
     0 (-b*L^2)/(2*J*r)];


%KALMAN FILTER CREATION
X = [0; 0; 0];
X_estimate = zeros(3, length(t)); % State vector estimate
X_stored = zeros(3, length(t));

U = [wR; wL];

P = eye(3,3).*10^2;

H = eye(3,3);

r_acc = 0.01; %variance
r_mag = 0.01; 
r_gyro = 0.01; 
R = [r_acc 0 0;
     0 r_mag 0;
     0 0 r_gyro];

F = [0 0 0;
     0 1 Ts;
     0 0 0];

G = [(pi*D)/2  (pi*D)/2;
      0    0;
     (pi*D)/L -(pi*D)/L];



phi = F;

stored_sqrt_P = zeros(3,3,length(t)); %Used to plot the expected estimation errors

process_noise = 0;
Q = process_noise*[ (Ts^5)/20 (Ts^4)/8 (Ts^3)/6;
                    (Ts^4)/8  (Ts^3)/3 (Ts^2)/2;
                    (Ts^3)/6  (Ts^2)/2  Ts      ]; %Process noise covariance matrix


test = zeros(3,length(t));

for k = 2:length(t)
    true_state(1,k) = (pi*D*(wR(k)+wL(k)))/2; %true Vb speed
    true_state(3,k) = (pi*D*(wR(k)-wL(k)))/L; %true spi_dot angular speed
    true_state(2,k) = true_state(2,k-1) + true_state(3,k-1)*Ts;
    
    Y(:,k) = [true_state(1,k)+randn*sqrt(r_acc);
              true_state(2,k)+randn*sqrt(r_mag);
              true_state(3,k)+randn*sqrt(r_gyro);];

    % Store the values of P for plotting
    % stored_sqrt_P(:,:,k) = sqrt(P);

    % UPDATE STEP
    % Kalman gain : K(k)
    K = P*H' / (H*P*H' + R);

    %Update state vecor estimate X^(k)
    X_estimate(:,k) = X_estimate(:,k-1) + K*(Y(:,k)-H*X_estimate(:,k-1)); 
    X_stored(:,k) = X_estimate(:,k);

    %Update state estimation error: P(k|k-1) to P(k|k)
    P = (eye(3) - K*H)*P;

    % PROJECTION STEP
    %project state estimation error: P(k|k) to P(k+1|k)
    P = phi*P*phi' + Q;
    
    %project state vecor estimation X^(k|k) to X^(k+1|k)
    X_estimate(:,k) = phi*X_estimate(:,k)  + G*U(:,k);
    % test(:,k) = G*U(:,k);
end


% --- Plot Wheel speeds ---
figure;
subplot(2,1,1); hold on; grid on;
plot(t, wR, 'g', 'DisplayName', 'true wR');
title('Right wheel rotational speed');
ylim([5.5 6.5]);
xlabel('Time [s]');
ylabel('Speed [Tr/min]');
legend;

subplot(2,1,2); hold on; grid on;
plot(t, wL, 'g', 'DisplayName', 'true wL');
title('Left wheel rotational speed');
ylim([5.5 6.5]);
xlabel('Time [s]');
ylabel('Speed [Tr/min]');
legend;


% ----- Plot Wheel speeds -----
figure;
subplot(3,1,1); hold on; grid on;
plot(t, true_state(1,:), 'g', 'DisplayName', 'true body speed');
plot(t, Y(1,:), 'r', 'DisplayName', 'IMU body speed');
plot(t, X_stored(1,:), 'b', 'DisplayName', 'Estimated body speed');
% plot(t, test(1,:), 'b', 'DisplayName', 'Estimated body speed');
title('Vb, body frame speed');
ylim([0 10]);
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend;

subplot(3,1,2); hold on; grid on;
plot(t, true_state(2,:), 'g', 'DisplayName', 'true body orientation');
plot(t, Y(2,:), 'r', 'DisplayName', 'IMU body orientation');
plot(t, X_stored(2,:), 'b', 'DisplayName', 'Estimated body orientation');
% plot(t, test(2,:), 'b', 'DisplayName', 'Estimated body speed');
title('Psi, body orientation');
ylim([-3 1]);
xlabel('Time [s]');
ylabel('Psi [rad]');
legend;

subplot(3,1,3); hold on; grid on;
plot(t, true_state(3,:), 'g', 'DisplayName', 'true body angular speed');
plot(t, Y(3,:), 'r', 'DisplayName', 'IMU body angular speed');
plot(t, X_stored(3,:), 'b', 'DisplayName', 'Estimated body angular speed');
% plot(t, test(3,:), 'b', 'DisplayName', 'Estimated body speed');
title('Psi_dot, body angular speed');
ylim([-0.65 0.5]);
xlabel('Time [s]');
ylabel('Psi_dot [rad/s]');
legend;


