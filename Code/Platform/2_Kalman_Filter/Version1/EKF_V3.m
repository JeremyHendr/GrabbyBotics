%% Differential Drive Robot Trajectory Simulation
clear; close all; clc;

% --- Robot parameters ---
L = 0.20;        % wheel separation [m]
Ts = 0.01;       % time step [s]
T = 10;          % total simulation time [s]
t = 0:Ts:T;
wheel_diameter = 0.05; % [m]
D = wheel_diameter;
global_speed = 1; %[m/s]
encoder_noise_deviation = 0.3; %[rev/sec]
m = 5; %mass of the robot
fr = 0.5;

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


%                  x
%                  x_dot
%                  y
% state_vector = [ y_dot   ]
%                  yaw
%                  yaw_dot


% --------------------------- True position calculation initialisation ---------------------------
state_truth = zeros(6, length(t));
yaw_truth = state_truth(5,1);
H_truth = [ pi*D*Ts*cos(yaw_truth)/2 pi*D*Ts*cos(yaw_truth)/2 ;
                  0                  0          ;
          pi*D*Ts*sin(yaw_truth)/2 pi*D*Ts*sin(yaw_truth)/2 ;
                  0                  0          ;
              pi*D*Ts/L         -pi*D*Ts/L      ;
                  0                  0          ];

% --------------------------- noisy estimation position initialisation ---------------------------
state_estimation = zeros(6, length(t));
yaw_estimation = state_estimation(5,1);
H_estimation = [ pi*D*Ts*cos(yaw_estimation)/2 pi*D*Ts*cos(yaw_estimation)/2 ;
                  0                  0          ;
          pi*D*Ts*sin(yaw_estimation)/2 pi*D*Ts*sin(yaw_estimation)/2 ;
                  0                  0          ;
              pi*D*Ts/L         -pi*D*Ts/L      ;
                  0                  0          ];




% --------------------------- EKF estimation initialisation ---------------------------
% We have issues with division by 0 when psi=0[pi] because we get sin(psi)=0
% This is why we use Vb = x_dot/cos(psi) or Vb = y_dot/sin(psi)

X = zeros(6, 1);
X_estimate = zeros(6, length(t)); % State vector estimate
X_estimate_stored = zeros(6, length(t)); %Used to store the values of X because X_estimate will store the projections using the math model ( X(k+1|k) )

% --- Input matrix H ---
psi = X_estimate(5,1);
if abs(mod(psi, pi)) < 1e-3
    a = 1/(pi*D*cos(psi));
    b = L/(2*pi*D);
    H = [0 a 0 0 0 b;
         0 a 0 0 0 -b];
else
    a = 1/(pi*D*sin(psi));
    b = L/(2*pi*D);
    H = [0 0 a 0 0 b;
         0 0 a 0 0 -b];
end


% --- Initial estimation error covariance matrix ---
P = diag(ones(size(X))).*(10);
stored_sqrt_P = zeros(length(X),length(X),length(t));

% --- measurement noise cov matrix ---
R = [encoder_noise_deviation^2 0;
     0 encoder_noise_deviation^2];

% --- sensor matrix ---
Y = [encoder_right; encoder_left];
% Y = [wR;wL];

%  --- Initialisation of F ---
% for now we are gonna consider c with a certain value but we dont know
% this value so later we need to replace it by and introduce process noise
% on that parameter to estimate c
F = zeros(6,6);
x_dot = X_estimate(2,1);
y_dot = X_estimate(4,1);
psi = X_estimate(5,1);
psi_dot = X_estimate(6,1);
if abs(mod(psi, pi)) < 1e-3
    a = -fr/m;
    b = psi_dot*x_dot;
    c = psi_dot;
    d = x_dot;
    F = [0 1 0 0 0 0;
         0 a 0 0 b 0;
         0 0 0 1 0 0;
         0 c 0 0 0 d;
         0 0 0 0 0 1;
         0 0 0 0 0 0;];
else
    a = -fr/(m*tan(psi)) - psi_dot;
    b = -fr/(m*(1+psi^2));
    c = -y_dot;
    d = -fr/m + psi_dot/tan(psi);
    e = (psi_dot*y_dot)/(1+psi^2);
    f = y_dot/tan(psi);
    F = [0 1 0 0 0 0;
         0 0 0 a b c;
         0 0 0 1 0 0;
         0 0 0 d e f;
         0 0 0 0 0 1;
         0 0 0 0 0 0;];
end

% --- Discrete time transition matrix ---
phi = eye(size(X)) + F.*Ts;

% --- Process noise covariance matrix ---
Q = zeros(6,6);

% --- Simulation loop ---
for k = 2:length(t) 
    % --- True position calculation ---
    yaw_truth = state_truth(5,k-1);
    H_truth = [ pi*D*Ts*cos(yaw_truth)/2 pi*D*Ts*cos(yaw_truth)/2;
                  0                  0;
          pi*D*Ts*sin(yaw_truth)/2 pi*D*Ts*sin(yaw_truth)/2;
                  0                  0;
              pi*D*Ts/L         -pi*D*Ts/L;
                  0                  0          ];
    state_truth(:,k) = state_truth(:,k-1) + H_truth * [wR(k); wL(k)];



    % --- noisy estimation position initialisation ---
    yaw_estimation = state_estimation(5,k-1);
    H_estimation = [ pi*D*Ts*cos(yaw_estimation)/2 pi*D*Ts*cos(yaw_estimation)/2 ;
                      0                  0          ;
                  pi*D*Ts*sin(yaw_estimation)/2 pi*D*Ts*sin(yaw_estimation)/2 ;
                          0                  0          ;
                      pi*D*Ts/L         -pi*D*Ts/L      ;
                          0                  0          ];
    state_estimation(:,k) = state_estimation(:,k-1) + H_estimation * [encoder_right(k); encoder_left(k)];

    
    
    % --- EKF estimation initialisation ---
    % Store the values of P for plotting
    stored_sqrt_P(:,:,k) = sqrt(P);
    
    % Kalman gain : K(k)
    K = P*H' / (H*P*H' + R);

    %Update state vecor estimate X(k|k)
    X_estimate(:,k) = X_estimate(:,k-1)+K*(Y(:,k)-H*X_estimate(:,k-1));
    X_estimate_stored(:,k) = X_estimate(:,k);



    % Update H(k)
    psi = X_estimate(5,k);
    if abs(mod(psi, pi)) < 1e-3
        a = 1/(pi*D*cos(psi));
        b = L/(2*pi*D);
        H = [0 a 0 0 0 b;
             0 a 0 0 0 -b];
    else
        a = 1/(pi*D*sin(psi));
        b = L/(2*pi*D);
        H = [0 0 a 0 0 b;
             0 0 a 0 0 -b];
    end

    % Compute state estimation error: P(k|k-1) to P(k|k)
    P = (eye(size(X)) - K*H)*P;
    
    % Update of non-linear F
    x_dot = X_estimate(2,k);
    y_dot = X_estimate(4,k);
    psi = X_estimate(5,k);
    psi_dot = X_estimate(6,k);
    if abs(mod(psi, pi)) < 1e-3
        a = -fr/m;
        b = psi_dot*x_dot;
        c = psi_dot;
        d = x_dot;
        F = [0 1 0 0 0 0;
             0 a 0 0 b 0;
             0 0 0 1 0 0;
             0 c 0 0 0 d;
             0 0 0 0 0 1;
             0 0 0 0 0 0;];
    else
        a = -fr/(m*tan(psi)) - psi_dot;
        b = -fr/(m*(1+psi^2));
        c = -y_dot;
        d = -fr/m + psi_dot/tan(psi);
        e = (psi_dot*y_dot)/(1+psi^2);
        f = y_dot/tan(psi);
        F = [0 1 0 0 0 0;
             0 0 0 a b c;
             0 0 0 1 0 0;
             0 0 0 d e f;
             0 0 0 0 0 1;
             0 0 0 0 0 0;];
    end
    
    % Upate phi 
    phi = eye(length(X),length(X)) + F*Ts;
    
    % Project state estimation error: P(k|k) to P(k+1|k)
    P = phi*P*phi' + Q;
    
    %project state vecor estimation X^(k|k) to X^(k+1|k)
    X_estimate(:,k) = phi*X_estimate(:,k);
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
% plot(state_truth(1,:), state_truth(3,:), 'g', 'LineWidth', 2, 'DisplayName', 'True position');
% plot(state_estimation(1,:), state_estimation(3,:), 'b', 'LineWidth', 2, 'DisplayName', 'Estimation using encode data');
plot(X_estimate_stored(1,:), X_estimate_stored(3,:), 'b', 'LineWidth', 2, 'DisplayName', 'Estimation EKF');
xlabel('X [m]'); ylabel('Y [m]');
% ylim([-2 2]); xlim([-2 20]); 
title('Differential Drive Robot Trajectory');
axis equal; grid on;



% --- Plot Wheel speeds ---
figure;
subplot(3,1,1); hold on; grid on;
% plot(t, wR, 'g', 'DisplayName', 'true wheel speed');
plot(t, X_estimate_stored(1,:), 'b', 'DisplayName', 'encoder data');
title('Right wheel speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend;

subplot(3,1,2); hold on; grid on;
% plot(t, wL, 'g', 'DisplayName', 'true wheel speed');
plot(t, X_estimate_stored(3,:), 'b', 'DisplayName', 'encoder data');
title('Right wheel speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend;

subplot(3,1,3); hold on; grid on;
% plot(t, wL, 'g', 'DisplayName', 'true wheel speed');
plot(t, X_estimate_stored(5,:), 'b', 'DisplayName', 'encoder data');
title('Right wheel speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend;

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


% quiver(X(1:100:end), Y(1:100:end), cos(TH(1:100:end)), sin(TH(1:100:end)), 0.2, 'r');