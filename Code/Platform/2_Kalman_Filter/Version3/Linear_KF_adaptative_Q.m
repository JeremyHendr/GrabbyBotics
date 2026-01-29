clear; clc; close all;
% ========== Initialize Variables ==========
% ------ Simulation ------
Ts = 1e-2;
t = 0:Ts:20;
N = length(t);          % number of points in the simulation
rpm_to_rad = (2*pi)/60; % coef to convert rpm to rad
rad_to_rpm = 60/(2*pi); % coef to convert rad to rpm

% ------ Motor ------
R_motor  = 2.14;     % [Ohm] motor resistance
Kt = 0.0038;         % [N·m/A] motor torque coefficient
Ke = 0.011;          % [V/(rad/s)] (output shaft)

% ------ Robot ------
gear_ratio = 98.78;
D = 0.1;    % [m] wheel diameter
r = D/2;    % [m] wheel radius
L = 0.20;   % [m] wheel separation
m = 5;      % [kg] mass of the robot
J = 0.1454; % [kg.m^2] moment of inertia of the robot (33*49 and 5kg)
b = 1.0;    % friction coefficient wheels


% ==================== System Input ====================
V1 = ones(2,N).*12;         % [V] Tension applied on both motors
V1(1, t >= 2 & t <= 4) = 8; % turning phase

V2 = ones(2,N).*12;
V2(1, t >= 2 & t <= 4) = 8;
V2(2, t >= 4 & t <= 6) = 8;

V3 = ones(2,N).*12;
V3(1, :) = 10 + 2.*cos(t);

V_truth = V3;
% ======================================================



% ========== Initialize State Matrices ==========
% ------ True Robot ------
W_wheels = zeros(2,N);           % [rad/s] Rotational speed of the wheels
T_wheels = zeros(2,N);           % [N.m] Torque of the wheels

X_truth = zeros(3,N);  % [v; yaw; yaw_dot;]

G_truth = [1/(m*r)   1/(m*r);
     0         0;
     L/(2*J*r) -L/(2*J*r)];

F_truth = [-(2*b)/(m*r)  0         0;
           0       0         1;
           0       0  -b*L^2/(2*J*r)];

inv_kynematics = [1/r 0 L/(2*r);
                  1/r 0 -L/(2*r)];

phi_true = eye(3) + F_truth*Ts;
Gd_true  = G_truth.*Ts;

% ------ True Motor ------
T_motors_true = zeros(2,N);
W_motors_true = zeros(2,N);
current_true = zeros(2,N);


% ========== First step calculation ==========
% ------ True state ------
W_wheels(:,1) = inv_kynematics*X_truth(:,1);

W_motors_true(:,1) = W_wheels(:,1)*gear_ratio;
current_true(:,1) = (V_truth(:,1) - Ke*W_motors_true(:,1))/R_motor;
T_motors_true(:,1) = Kt*current_true(1);

X_truth(:,1) = phi_true*X_truth(:,1) + Gd_true*T_wheels(:,1);

X_inv_kin = zeros(4,N);
X_inv_kin(1,1) = (W_wheels(1,1)+W_wheels(2,1))*(r/2);
X_inv_kin(3,1) = (W_wheels(1,1)-W_wheels(2,1))*(r/L);
X_inv_kin(2,1) = X_inv_kin(2,1)+X_inv_kin(3,1)*Ts;
X_inv_kin(4,1) = X_inv_kin(4,1)+X_inv_kin(3,1)*Ts;


% ========== Loop calculation ==========
for k = 2:N
    % ------ Perform true calcutation of the state ------
    W_wheels(:,k) = inv_kynematics*X_truth(:,k-1);

    W_motors_true(:,k) = W_wheels(:,k) .* gear_ratio;
    current_true(:,k) = (V_truth(:,k) - W_motors_true(:,k).*Ke) ./ R_motor;
    T_motors_true(:,k) = current_true(:,k).*Kt;
    T_wheels(:,k) = T_motors_true(:,k).*gear_ratio;

    X_truth(:,k) = phi_true*X_truth(:,k-1) + Gd_true*T_wheels(:,k);
    

    % --- Perform true calculation of the state using inverse kinematics ---
    X_inv_kin(1,k) = (W_wheels(1,k)+W_wheels(2,k))*(r/2); % True speed
    X_inv_kin(3,k) = (W_wheels(1,k)-W_wheels(2,k))*(r/L); % True angular speed
    X_inv_kin(2,k) = X_inv_kin(2,k-1)+X_inv_kin(3,k)*Ts;  % True angle
    X_inv_kin(4,k) = (X_inv_kin(3,k)-X_inv_kin(3,k-1))/Ts;% True angular acceleration  
end

% ========== Plotting ==========
% ------ Motor ------
figure; set(gcf, 'Name', 'Motor Data', 'NumberTitle', 'off');
% Speed plot
subplot(3,1,1);
plot(t, W_wheels(1,:).*rad_to_rpm, 'b'); hold on;
plot(t, W_wheels(2,:).*rad_to_rpm, 'r'); hold on;
title('Wheel speed');
xlabel('Time [s]');
ylabel('Rotational speed [RPM]');
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
plot(t, current_true(1,:), 'r', 'LineWidth', 2); hold on;
plot(t, current_true(2,:), 'b', 'LineWidth', 2); hold on;
title('Motor current');
xlabel('Time [s]');
ylabel('Ampere [A]');
grid on;

% ------ Robot ------
figure; set(gcf, 'Name', 'True robot state', 'NumberTitle', 'off');
% Speed plot
subplot(3,1,1);
plot(t, X_truth(1,:), 'r'); hold on;
plot(t, X_inv_kin(1,:), 'b'); 
ylabel('Speed [m/s]');
title('Robot speed');
xlabel('Time [s]');
grid on; legend('Dynamic equations','Inverse kinematics');

% Angular speed
subplot(3,1,2);
plot(t, X_truth(2,:), 'r', 'LineWidth', 2); hold on;
plot(t, X_inv_kin(2,:), 'b', 'LineWidth', 2);
title('Robot angle');
xlabel('Time [s]');
ylabel('angle [rad]');
grid on; legend('Dynamic equations','Inverse kinematics');

% Robot angular acceleration
subplot(3,1,3);
plot(t, X_truth(3,:), 'r', 'LineWidth', 2); hold on;
plot(t, X_inv_kin(3,:), 'b', 'LineWidth', 2);
title('Robot angular speed');
xlabel('Time [s]');
ylabel('angular speed [rad/s]');
grid on; legend('Dynamic equations','Inverse kinematics');




%% |||||||||||||||||||||||||| KALMAN FILTER IMPLEMENTATION |||||||||||||||||||||||||| 

% --------------- Sensor data generation ---------------
acc_deviation_x = 0.01;
acc_deviation_yaw = 0.05;
acc_deviation_yaw_dot = 0.01;
acc_sensor = X_truth + [randn(1,N).*acc_deviation_x;
                        randn(1,N).*acc_deviation_yaw;
                        randn(1,N).*acc_deviation_yaw_dot;];


encoder_deviation = 10;
encoder_data = W_motors_true+[randn(1,N).*encoder_deviation;
                              randn(1,N).*encoder_deviation];
% % Slip: slip is not white Gaussian noise — treat it with adaptive R, augmented slip state, or outlier rejection.
encoder_data(1, 3<=t & t<=4) = encoder_data(1, 3<=t & t<=4).*1.3; %introducing slip
encoder_data(1, 10<=t & t<=14) = encoder_data(1, 10<=t & t<=14).*0.9; %introducing slip

current_estimation = (V_truth - encoder_data.*Ke)./R_motor;
T_motors_estimation = current_estimation.*Kt;





% --------------- Kalman Filter ---------------
Y = acc_sensor; % sensor data used for prediction step
H = eye(3,3);

U = T_motors_estimation.*gear_ratio; % sensor data for update step

X = zeros(3, N);
X_stored = zeros(3, N);

P = eye(3,3).*10^9;

G = [1/(m*r)   1/(m*r);
        0         0;
     L/(2*J*r) -L/(2*J*r)];

F = [-(2*b)/(m*r)  0         0;
           0       0         1;
           0       0  -b*L^2/(2*J*r)];

phi = eye(3) + F*Ts;
Gd  = G.*Ts;

stored_sqrt_P = zeros(3,3,length(t)); %Used to plot the expected estimation errors

% process_noise = 0.1;
% Q = process_noise*[ (Ts^5)/20 (Ts^4)/8 (Ts^3)/6;
%                     (Ts^4)/8  (Ts^3)/3 (Ts^2)/2;
%                     (Ts^3)/6  (Ts^2)/2  Ts      ]; %Process noise covariance matrix

Q_coef = 100;
Q = Q_coef*[ Ts*acc_deviation_x^2           0                               0;
             0          acc_deviation_yaw*(Ts^3)/3                          acc_deviation_yaw*acc_deviation_yaw_dot*(Ts^2)/2;
             0          acc_deviation_yaw*acc_deviation_yaw_dot*(Ts^2)/2    acc_deviation_yaw_dot*Ts];


R = [acc_deviation_x^2      0                0;
     0             acc_deviation_yaw^2       0;
     0                      0      acc_deviation_yaw_dot^2];


NIS_store = zeros(1,N);
window = 100;  % number of samples for moving average
NIS_ma = zeros(1, N);
m = size(Y,1);  % measurement dimension

scale = ones(1,N);

% innov_store = zeros(3,N);
% ========== Loop calculation ==========
for k = 2:N
    % --- Kalman Filter ---
    % Store the values of P for plotting
    stored_sqrt_P(:,:,k) = sqrt(P);

    % UPDATE STEP
    % Kalman gain : K(k)
    K = P*H' / (H*P*H' + R);

    % Update state vecor estimate X^(k)
    X_stored(:,k) = X(:,k-1) + K*(Y(:,k)-H*X(:,k-1)); 

    % Update state estimation error: P(k|k-1) to P(k|k)
    P = (eye(3) - K*H)*P;
    
    innov = Y(:,k) - H*X(:,k-1);    % Y is 3*1 so innov to 
    S = H*P*H' + R;                 % R is 3*3 so S to
    NIS = innov' * (S \ innov);     % scalar
    NIS_store(k) = NIS;
    
    if k > window
        NIS_ma(k) = mean(NIS_store(k-window+1:k));
    else
        NIS_ma(k) = mean(NIS_store(1:k));
    end

    alpha = 0.02;   % adaptation rate (small)
    % alpha = 0.05;   % adaptation rate (medium)
    scale(k) = 1 + alpha*(NIS_ma(k) - m);

    % Clamp scaling to avoid exploding Q
    % scale(k) = min(max(scale(k), 0.8), 1.2);
    Q = Q .* scale(k);

    % innov_store(:,k) = innov;

    % PROJECTION STEP
    % project state estimation error: P(k|k) to P(k+1|k)
    P = phi*P*phi' + Q;

    % project state vecor estimation X^(k|k) to X^(k+1|k)
    X(:,k) = phi*X_stored(:,k)  + Gd*U(:,k);
end



%% ==================== Plotting ====================
% ------ Accelerometer data ------
figure; set(gcf, 'Name', 'Accelerometer data', 'NumberTitle', 'off');
% Speed plot
subplot(3,1,1);
plot(t, Y(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, X_truth(1,:), 'r', 'LineWidth', 0.5); hold on;
title('Robot speed'); ylabel('Speed [m/s]'); xlabel('Time [s]');
grid on; legend('Truth','Accelerometer'); hold off;

% Angular speed
subplot(3,1,2);
plot(t, Y(2,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, X_truth(2,:), 'r', 'LineWidth', 0.5); hold on;
title('Robot angle'); xlabel('Time [s]'); ylabel('angle [rad]');
grid on; legend('Truth','Accelerometer'); hold off;

% Robot angular acceleration
subplot(3,1,3);
plot(t, Y(3,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, X_truth(3,:), 'r', 'LineWidth', 0.5); hold on;
title('Robot angular speed'); xlabel('Time [s]'); ylabel('angular speed [rad/s]');
grid on; legend('Truth','Accelerometer'); hold off;


% ------ Estimation of torque data using encoder data ------
figure; set(gcf, 'Name', 'Encoder data', 'NumberTitle', 'off');
% Wheel speed plot
subplot(3,1,1);
plot(t, encoder_data(1,:)./gear_ratio.*rad_to_rpm, 'g', 'LineWidth', 1.5); hold on;
plot(t, encoder_data(2,:)./gear_ratio.*rad_to_rpm, 'y', 'LineWidth', 1.5); hold on;
plot(t, W_wheels(1,:).*rad_to_rpm, 'r', 'LineWidth', 0.5); hold on;
plot(t, W_wheels(2,:).*rad_to_rpm, 'b', 'LineWidth', 0.5); hold on;
title('Encoder data (wheel speed)'); xlabel('Time [s]'); ylabel('Rotational speed [RPM]');
legend('right estimation', 'left estimation','Right true', 'left true');
grid on; hold off;

% Torque plot
subplot(3,1,2);
plot(t, U(1,:), 'g', 'LineWidth', 1.5); hold on;
plot(t, U(2,:), 'y', 'LineWidth', 1.5); hold on;
plot(t, T_wheels(1,:), 'r', 'LineWidth', 0.5); hold on;
plot(t, T_wheels(2,:), 'b', 'LineWidth', 0.5); hold on;
title('Wheel Output torque estimation'); xlabel('Time [s]'); ylabel('Torque [N.m]');
legend('right estimation', 'left estimation','Right true', 'left true');
grid on; hold off;

% Current plot
subplot(3,1,3);
plot(t, current_estimation(1,:), 'g', 'LineWidth', 1.5); hold on;
plot(t, current_estimation(2,:), 'y', 'LineWidth', 1.5); hold on;
plot(t, current_true(1,:), 'r', 'LineWidth', 0.5);       hold on;
plot(t, current_true(2,:), 'b', 'LineWidth', 0.5);       hold on;
title('Motor current estimation'); xlabel('Time [s]'); ylabel('Ampere [A]');
legend('right estimation', 'left estimation','Right true', 'left true');
grid on; hold off;

% ------ Robot state estimation ------
figure; set(gcf, 'Name', 'KF estimation', 'NumberTitle', 'off');
% Speed plot
subplot(3,1,1);
plot(t, X_truth(1,:), 'r'); hold on;
plot(t, X_stored(1,:), 'b'); 
title('Robot speed'); xlabel('Time [s]'); ylabel('Speed [m/s]');
grid on; legend('True state','KF estimation');

% Angular speed
subplot(3,1,2);
plot(t, X_truth(2,:), 'r', 'LineWidth', 2); hold on;
plot(t, X_stored(2,:), 'b', 'LineWidth', 2);
title('Robot angle'); xlabel('Time [s]'); ylabel('angle [rad]');
grid on; legend('True state','KF estimation');

% Robot angular acceleration
subplot(3,1,3);
plot(t, X_truth(3,:), 'r', 'LineWidth', 2); hold on;
plot(t, X_stored(3,:), 'b', 'LineWidth', 2);
title('Robot angular speed'); xlabel('Time [s]'); ylabel('angular speed [rad/s]');
grid on; legend('True state','KF estimation');


% ========== Compute and plot robot 2D trajectory ==========
x_pos_true = zeros(1, N);
y_pos_true = zeros(1, N);

x_pos_KF = zeros(1, N);
y_pos_KF = zeros(1, N);

for k = 2:N
    x_pos_true(k) = x_pos_true(k-1) + X_truth(1,k) * cos(X_truth(2,k)) * Ts;
    y_pos_true(k) = y_pos_true(k-1) + X_truth(1,k) * sin(X_truth(2,k)) * Ts;

    x_pos_KF(k) = x_pos_KF(k-1) + X_stored(1,k) * cos(X_stored(2,k)) * Ts;
    y_pos_KF(k) = y_pos_KF(k-1) + X_stored(1,k) * sin(X_stored(2,k)) * Ts;
end


% Time markers every 1 second
t_labels = 0:1:t(end);
idx = round(t_labels / Ts) + 1;

% ---------- Plot the trajectory ----------
figure; set(gcf, 'Name', 'Navigation frame position', 'NumberTitle', 'off');
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

nexttile([3 1]);
plot(x_pos_true, y_pos_true, 'r', 'LineWidth', 2); hold on;
plot(x_pos_KF, y_pos_KF, 'b', 'LineWidth', 2); hold on;
plot(x_pos_true(idx), y_pos_true(idx), 'wx', 'MarkerSize', 6, 'LineWidth', 1.2); % Plot crosses and labels
text(x_pos_true(idx)+0.01, y_pos_true(idx)+0.01, compose('%.0fs', t_labels), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'w');
title('Robot Trajectory in 2D'); xlabel('x [m]'); ylabel('y [m]');
legend('True position', 'KF estimation', 'time marker');
ylim([-3.5 1]); xlim([-1 3]);
grid on;

nexttile;
plot(t, V_truth(1,:), 'r', 'LineWidth', 2); hold on;
plot(t, V_truth(2,:), 'b', 'LineWidth', 2);
ylim([-0.5 13])
title('Tension on motors'); xlabel('Time [s]'); ylabel('Tension [V]');
grid on; legend('Right','Left');





% ========== Estimation error plot ==========
%Position Estimation error
figure;
subplot(3,1,1); hold on; grid on;
plot(t, squeeze(stored_sqrt_P(1,1,:)), 'b', 'DisplayName', 'Upper expected position deviation');
plot(t, -squeeze(stored_sqrt_P(1,1,:)), 'b', 'DisplayName', 'Lower expected position deviation');
plot(t, X_truth(1,:)-X_stored(1,:), 'g', 'DisplayName', 'Speed error');
% ylim([-stored_sqrt_P(1,1,end)*3 stored_sqrt_P(1,1,end)*3]);
ylim([-0.02 0.02]);
title('Expedcted speed error compared to the actual error'); xlabel('Time [s]'); ylabel('Error [m]');
legend;

%Speed Estimation error
subplot(3,1,2); hold on; grid on;
plot(t, squeeze(stored_sqrt_P(2,2,:)), 'b', 'DisplayName', 'Upper expected speed deviation');
plot(t, -squeeze(stored_sqrt_P(2,2,:)), 'b', 'DisplayName', 'Upper expected speed deviation');
plot(t, X_truth(2,:)-X_stored(2,:), 'g', 'DisplayName', 'Orientation error');
ylim([-stored_sqrt_P(2,2,end)*3 stored_sqrt_P(2,2,end)*3]);
title('Expected speed error compared to the actual error'); xlabel('Time [s]'); ylabel('Error [m/s]');
legend;

%Acceleration Estimation error
subplot(3,1,3); hold on; grid on;
plot(t, squeeze(stored_sqrt_P(3,3,:)), 'b', 'DisplayName', 'Upper expected speed deviation');
plot(t, -squeeze(stored_sqrt_P(3,3,:)), 'b', 'DisplayName', 'Upper expected speed deviation');
plot(t, X_truth(3,:)-X_stored(3,:), 'g', 'DisplayName', 'Angular speed error');
ylim([-stored_sqrt_P(3,3,end)*3 stored_sqrt_P(3,3,end)*3]);
title('Expected Aceeleration error compared to the actual error'); xlabel('Time [s]'); ylabel('Error [m/s^2]');
legend;


% Expected chi-square mean for number of measurements (m)
m = size(Y,1);
chi2_mean = m;                      % expected mean NIS
chi2_95 = chi2inv(0.95, m);         % 95% upper bound
chi2_05 = chi2inv(0.05, m);         % 5% lower bound

figure('Name', 'NIS and Innovation Magnitude');
subplot(2,1,1);
plot(t, NIS_store, 'b', 'LineWidth', 1.5); hold on;
plot(t, NIS_ma, 'g', 'LineWidth', 2 ); hold on;
yline(chi2_mean, '--w', 'Mean NIS');
yline(chi2_95, '--r', '95% bound');
yline(chi2_05, '--r', '5% bound');
ylim([0 15]);
xlabel('Time [s]'); ylabel('NIS');
title('Normalized Innovation Squared (Consistency Check)');
legend('NIS', 'Mean', '95%/5% bounds');
grid on;

subplot(2,1,2);
plot(t, scale, 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Scale for Q');
title('Innovation Magnitude (Measurement Residual)');
grid on;


% ✅ If most NIS values lie between the red bounds (95% interval), your filter is statistically consistent.
% 🔴 If NIS is systematically above the upper bound → model is too confident → increase Q.
% 🔵 If NIS is systematically below the lower bound → model too uncertain → decrease Q or R.
% 
% NIS has an expected value equal to the dimension of the measurement vector, 
% 𝑚=size(𝑌,1), if the filter is correctly tuned and consistent.