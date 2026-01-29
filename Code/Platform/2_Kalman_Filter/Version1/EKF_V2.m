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
c = 0.5;

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


% --- True position calculation initialisation ---
state_truth = zeros(6, length(t));
yaw_truth = state_truth(5,1);
H_truth = [ pi*D*Ts*cos(yaw_truth)/2 pi*D*Ts*cos(yaw_truth)/2 ;
                  0                  0          ;
          pi*D*Ts*sin(yaw_truth)/2 pi*D*Ts*sin(yaw_truth)/2 ;
                  0                  0          ;
              pi*D*Ts/L         -pi*D*Ts/L      ;
                  0                  0          ];

% --- noisy estimation position initialisation ---
state_estimation = zeros(6, length(t));
yaw_estimation = state_estimation(5,1);
H_estimation = [ pi*D*Ts*cos(yaw_estimation)/2 pi*D*Ts*cos(yaw_estimation)/2 ;
                  0                  0          ;
          pi*D*Ts*sin(yaw_estimation)/2 pi*D*Ts*sin(yaw_estimation)/2 ;
                  0                  0          ;
              pi*D*Ts/L         -pi*D*Ts/L      ;
                  0                  0          ];

% --- EKF estimation initialisation ---
X = zeros(6, 1);
X_estimate = zeros(6, length(t)); % State vector estimate
X_estimate_stored = zeros(6, length(t)); %Used to store the values of X because X_estimate will store the projections using the math model ( X(k+1|k) )



%Input matrix (non-linear)
yaw = X_estimate(5,1);
vx = X_estimate(2,1);
vy = X_estimate(4,1);
H = [0 cos(yaw) 0 sin(yaw) (-sin(yaw)*vx+cos(yaw)*vy) L/2;
     0 cos(yaw) 0 sin(yaw) (-sin(yaw)*vx+cos(yaw)*vy) -L/2];

% H = [0 1/(pi*D*cos(yaw)) 0 0 0 (pi*D)/L;
%      0 0 0 1/(pi*D*sin(yaw)) 0 (pi*D)/L];


% Initial estimation error covariance matrix
P = diag(ones(size(X))).*(10^9);
stored_sqrt_P = zeros(length(X),length(X),length(t));

% measurement noise cov matrix
R = [encoder_noise_deviation^2 0;
     0 encoder_noise_deviation^2];

% sensor matrix
sensor = [encoder_right; encoder_left];

% Update of non-linear f
x_dot = X_estimate(2,1);
yaw = X_estimate(5,1);
yaw_dot = X_estimate(6,1);
a = -(c/m + tan(yaw)*yaw_dot);
b = (yaw_dot*x_dot)/cos(yaw)^2;
c = tan(yaw)*x_dot;
d = -(c/m - atan(yaw)*yaw_dot);
e = (yaw_dot*x_dot)/(1+yaw^2);
f = atan(yaw)*yaw_dot;
F = [ 0 1 0 0 0 0;
      0 a 0 0 b c;
      0 0 0 1 0 0;
      0 0 0 d e f;
      0 0 0 0 0 1;
      0 0 0 0 0 0];

% Discrete time transition matrix
phi = eye(size(X)) + F.*Ts;

%Process noise covariance matrix
process_noise = 0;
Q = process_noise*[ (Ts^5)/20 (Ts^4)/8 (Ts^3)/6;
                    (Ts^4)/8  (Ts^3)/3 (Ts^2)/2;
                    (Ts^3)/6  (Ts^2)/2  Ts      ];
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
    
    %update H
    yaw = X_estimate(5,k);
    vx = X_estimate(2,k);
    vy = X_estimate(4,k);
    H = [0 cos(yaw) 0 sin(yaw) (-sin(yaw)*vx+cos(yaw)*vy) L/2;
         0 cos(yaw) 0 sin(yaw) (-sin(yaw)*vx+cos(yaw)*vy) -L/2];

    % Kalman gain : K(k)
    K = P*H' / (H*P*H' + R);

    %Update state vecor estimate X^(k)
    X_estimate(:,k) = X_estimate(:,k-1)+K*(sensor(:,k)-H*X_estimate(:,k-1));
    X_stored(:,k) = X_estimate(:,k);

    %Update state estimation error: P(k|k-1) to P(k|k)
    P = (eye(size(X)) - K*H)*P;
    
    % Update of non-linear F
    x_dot = X_estimate(2,k);
    yaw = X_estimate(5,k);
    yaw_dot = X_estimate(6,k);
    a = -(c/m + tan(yaw)*yaw_dot);
    b = (yaw_dot*x_dot)/cos(yaw)^2;
    c = tan(yaw)*x_dot;
    d = -(c/m - atan(yaw)*yaw_dot);
    e = (yaw_dot*x_dot)/(1+yaw^2);
    f = atan(yaw)*yaw_dot;
    F = [ 0 1 0 0 0 0;
          0 a 0 0 b c;
          0 0 0 1 0 0;
          0 0 0 d e f;
          0 0 0 0 0 1;
          0 0 0 0 0 0];
    
    phi = eye(size(X)) + F.*Ts;
    
    %project state estimation error: P(k|k) to P(k+1|k)
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
plot(X_stored(1,:), X_stored(3,:), 'b', 'LineWidth', 2, 'DisplayName', 'Estimation EKF');
xlabel('X [m]'); ylabel('Y [m]');
% ylim([-2 2]); xlim([-2 20]); 
title('Differential Drive Robot Trajectory');
axis equal; grid on;






% quiver(X(1:100:end), Y(1:100:end), cos(TH(1:100:end)), sin(TH(1:100:end)), 0.2, 'r');