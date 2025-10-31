%% CART-PENDULUM COMPLETE SYSTEM - LQR CONTROLLER DESIGN
% Complete system with velocity command as input
% Renan Duarte - 30/10/2025

format long eng;
clear all;
close all;
clc;

% Search stuff also in this folder
addpath ('./Datasources/');
addpath ('./Functions/');
addpath ('./Results/');

% Start counting time
tic;

%% LOAD OPTIMIZED PARAMETERS

% Load cart parameters
try
    load('./Results/Cart.mat', 'Cart');
    fprintf('Cart parameters loaded successfully\n');
catch
    fprintf('Cart parameters not found. Aborting.\n');
    return
end

% Load pendulum parameters
try
    load('./Results/Pendulum.mat', 'Pendulum', 'PendulumLinear');
    fprintf('Pendulum parameters loaded successfully\n\n');
catch
    fprintf('Pendulum parameters not found. Aborting.\n\n');
    return;
end

% Display loaded parameters
fprintf('LOADED PARAMETERS\n\n');
fprintf('Cart (Linear):\n');
fprintf('  tau  = %.6f s\n', Cart.Tr);
fprintf('  Kv   = %.4f\n', Cart.Kv);
fprintf('  Kp   = %.4f\n\n', Cart.Kp);

fprintf('Pendulum (Linear):\n');
fprintf('  m    = %.6f kg\n', PendulumLinear.m);
fprintf('  l    = %.6f m\n', PendulumLinear.l);
fprintf('  kd   = %.6e Ns/m\n\n', PendulumLinear.kd);

%% BUILD LINEAR STATE-SPACE MODEL

fprintf('BUILDING LINEAR STATE-SPACE MODEL...\n\n');

% Cart linear parameters
tau = Cart.Tr;
Kv = Cart.Kv;
Kp = Cart.Kp;

% Pendulum linear parameters
m = PendulumLinear.m;
l = PendulumLinear.l;
kd = PendulumLinear.kd;

% Constants
g = 9.81;  % [m/s^2]
I = m * l^2;

% State-space matrices for complete linearized system
% States: [p, v, theta, omega]'
% Input: Vcmd (velocity command)
% Output: [p, v, theta, omega]'

% Determine linearization point based on pendulum configuration
PendulumUp = true;
if PendulumUp == true
    % Pendulum up: equilibrium at theta = 0
    cos_theta = 1;
    dsin_dtheta = 1;
else
    % Pendulum down: equilibrium at theta = pi
    cos_theta = -1;
    dsin_dtheta = -1;
end

% State-Space Matrices
A = [0, Kp,                     0,                      0;
     0, -1/tau,                 0,                      0;
     0, 0,                      0,                      1;
     0, m*l*cos_theta/(I*tau), m*g*l*dsin_dtheta/I,    -kd/I];

B = [0; 
     Kv/tau; 
     0; 
     -m*l*cos_theta*Kv/(I*tau)];

C = eye(4);

D = zeros(4, 1);

% Create continuous-time state-space system
sys_c = ss(A, B, C, D);

fprintf('Continuous-time state-space model:\n');
fprintf('System order: %d states\n', size(A, 1));
fprintf('Number of inputs: %d\n', size(B, 2));
fprintf('Number of outputs: %d\n\n', size(C, 1));

fprintf('A matrix:\n');
disp(A);
fprintf('B matrix:\n');
disp(B);

%% DISCRETIZATION

% Sample time
Ts = 1/200;  % 200 Hz
fprintf('Sample time: %.6f s (%.1f Hz)\n\n', Ts, 1/Ts);

% Discretize using zero-order hold
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

fprintf('Discrete-time state-space model created\n\n');

%% LQR DESIGN

fprintf('LQR CONTROLLER DESIGN\n\n');

% LQR weighting matrices
% Q: State penalty matrix (penalize deviations in states)
%    [p, v, phi, phi_dot]
% Higher values = more penalty on that state
Q = diag([1,  % Cart position
          0,    % Cart velocity
          20,  % Pendulum angle
          0]);  % Pendulum angular velocity

% R: Input penalty matrix (penalize large control efforts)
% Higher value = less aggressive control
R = 100;

fprintf('LQR weighting matrices:\n');
fprintf('Q = diag([%.1f, %.1f, %.1f, %.1f])\n', Q(1,1), Q(2,2), Q(3,3), Q(4,4));
fprintf('R = %.2f\n\n', R);

% Compute LQR gain
K = dlqr(Ad, Bd, Q, R);

fprintf('LQR Gains:\n');
fprintf('K = [%.6f, %.6f, %.6f, %.6f]\n\n', K(1), K(2), K(3), K(4));

%% FEEDFORWARD GAIN FOR REFERENCE TRACKING

fprintf('FEEDFORWARD GAIN CALCULATION\n\n');

% Closed-loop system matrix
Acl = Ad - Bd*K;

% Calculate Kr for zero steady-state error
% We want only position tracking (first output), so use only first row of Cd
Cd_pos = Cd(1,:);  % [1 0 0 0] - only position output

% Kr calculation for position reference tracking
% Formula: Kr = 1 / (Cd_pos * inv(I - Acl) * Bd)
Kr = 1 / (Cd_pos * inv(eye(size(Acl)) - Acl) * Bd);

fprintf('Feedforward gain for position tracking:\n');
fprintf('Kr = %.6f\n\n', Kr);

%% POLE ANALYSIS

fprintf('POLE LOCATIONS\n\n');

% Open-loop poles
poles_ol = eig(Ad);
fprintf('Open-loop poles (discrete):\n');
for i = 1:length(poles_ol)
    fprintf('  p%d = %.6f %+.6fi  (magnitude: %.6f)\n', ...
            i, real(poles_ol(i)), imag(poles_ol(i)), abs(poles_ol(i)));
end

% Stability check
if all(abs(poles_ol) < 1)
    fprintf('✓ System is STABLE (all poles inside unit circle)\n\n');
else
    fprintf('✗ System is UNSTABLE (poles outside unit circle)\n\n');
end

% Closed-loop poles
poles_cl = eig(Acl);
fprintf('Closed-loop poles (discrete):\n');
for i = 1:length(poles_cl)
    fprintf('  p%d = %.6f %+.6fi  (magnitude: %.6f)\n', ...
            i, real(poles_cl(i)), imag(poles_cl(i)), abs(poles_cl(i)));
end
fprintf('\n');

% Stability check
if all(abs(poles_cl) < 1)
    fprintf('✓ System is STABLE (all poles inside unit circle)\n\n');
else
    fprintf('✗ System is UNSTABLE (poles outside unit circle)\n\n');
end

%% CLOSED-LOOP SIMULATION WITH NON-LINEAR MODEL

fprintf('SIMULATING CLOSED-LOOP SYSTEM WITH NON-LINEAR MODEL...\n\n');

% Simulation time
Time = 0:Ts:3;
N = length(Time);

% Initial condition (pendulum slightly off vertical)
xi = [0;        % Cart position [m]
      0;        % Cart velocity [m/s]
      -0.1;     % Pendulum angle deviation from upward [rad] (~5.7 deg)
      0];       % Pendulum angular velocity [rad/s]

% Reference state (all zeros - cart at origin, pendulum upright)
ref = [0; 0; 0; 0];

% Preallocate arrays
States = zeros(N, 4);
States(1,:) = xi';
Vcmd = zeros(N, 1);
Accel = zeros(N, 1);

% Non-linear model parameters
Params_NL = [Cart.Amax, Cart.Tr, Cart.Kv, Cart.Kp, ...
             Pendulum.m, Pendulum.l, Pendulum.kd, Pendulum.kdr, Pendulum.kc];

% Simulation loop with ZOH (input held constant between samples)
for i = 1:N-1
    % LQR control law with feedforward: u = Kr*r - K*(x - [r; 0; 0; 0])
    % Simplified: u = Kr*r - K*x + K*[r; 0; 0; 0]
    %           = Kr*r - K*x + K(1)*r  (since only first element of ref is non-zero)
    %           = (Kr + K(1))*r - K*x   
    Vcmd(i) = Kr * ref(1) - K * States(i,:)';
    
    % Calculate cart acceleration (same logic as in the nonlinear model)
    v = States(i, 2);
    v_error = Kv * Vcmd(i) - v;
    Accel(i) = sign(v_error) * min(abs(v_error) / tau, Cart.Amax);
    
    % Simulate non-linear model for one time step with constant input
    tspan = [Time(i), Time(i+1)];
   
    [~, y_temp] = ode45(@(t, y) CartPendNonLinearModel(t, y, Params_NL, Vcmd(i), Time(i)), ...
                        tspan, States(i,:)');
    
    % Next state
    States(i+1,:) = y_temp(end,:);
end

% Extract states for plotting
PosM = States(:,1);
VelM = States(:,2);
ThetaM = States(:,3);
ThetaDotM = States(:,4);

Time_Duration = toc;
fprintf('Simulation completed in %.2f seconds\n\n', Time_Duration);

%% PERFORMANCE METRICS

fprintf('PERFORMANCE METRICS\n\n');

% Define settling criterion (2%)
settling_criterion = 0.02;

% For position - settling to reference (0 m)
pos_ref = ref(1);  % = 0
pos_tolerance = 0.01;  % 1 cm tolerance (absolute, since ref = 0)

% Find where position stays within tolerance
pos_error = abs(PosM - pos_ref);
pos_settled_idx = find(pos_error <= pos_tolerance, 1, 'first');

if isempty(pos_settled_idx)
    pos_settling_time = inf;
else
    % Verify it stays settled (doesn't leave the band again)
    if all(pos_error(pos_settled_idx:end) <= pos_tolerance)
        pos_settling_time = Time(pos_settled_idx);
    else
        % Find the last time it enters the settling band
        temp_idx = find(pos_error > pos_tolerance, 1, 'last');
        if isempty(temp_idx)
            pos_settling_time = Time(pos_settled_idx);
        elseif temp_idx >= numel(Time)
            pos_settling_time = inf;
        else
            pos_settling_time = Time(temp_idx + 1);
        end
    end
end

% For angle - settling to reference (0 rad)
theta_ref = ref(3);  % = 0
theta_tolerance_deg = 1.0;  % 1 degree
theta_tolerance = theta_tolerance_deg * pi/180;  % Convert to radians

% Find where angle stays within tolerance
theta_error = abs(ThetaM - theta_ref);
theta_settled_idx = find(theta_error <= theta_tolerance, 1, 'first');

if isempty(theta_settled_idx)
    theta_settling_time = inf;
else
    % Verify it stays settled
    if all(theta_error(theta_settled_idx:end) <= theta_tolerance)
        theta_settling_time = Time(theta_settled_idx);
    else
        % Find the last time it enters the settling band
        temp_idx = find(theta_error > theta_tolerance, 1, 'last');
        if isempty(temp_idx)
            theta_settling_time = Time(theta_settled_idx);
        else
            theta_settling_time = Time(temp_idx + 1);
        end
    end
end

fprintf('Settling time criteria:\n');
fprintf('  Position tolerance: %.3f m\n', pos_tolerance);
fprintf('  Angle tolerance: %.2f deg (%.4f rad)\n', theta_tolerance_deg, theta_tolerance);
fprintf('\nSettling times:\n');
fprintf('  Position: %.3f s\n', pos_settling_time);
fprintf('  Angle: %.3f s\n\n', theta_settling_time);

fprintf('Final state errors (at t=%.1f s):\n', Time(end));
fprintf('  Position: %.6f m\n', abs(PosM(end) - pos_ref));
fprintf('  Angle: %.6f rad (%.4f deg)\n\n', abs(ThetaM(end) - theta_ref), abs(ThetaM(end) - theta_ref)*180/pi);

% Maximum deviations
fprintf('Maximum deviations:\n');
fprintf('  Position: %.6f m\n', max(abs(PosM - pos_ref)));
fprintf('  Angle: %.6f rad (%.4f deg)\n\n', max(abs(ThetaM - theta_ref)), max(abs(ThetaM - theta_ref))*180/pi);

%% PLOTS
hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% LEFT SIDE - STATES (4 rows)

% Cart position
subplot(4,2,1);
plot(Time, PosM);
hold on;
plot(Time, pos_ref*ones(numel(1,Time)), 'r--');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title(sprintf('Cart Position (settling: %.2f s)', pos_settling_time));

% Cart velocity
subplot(4,2,3);
plot(Time, VelM);
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Cart Velocity');

% Pendulum angle
subplot(4,2,5);
plot(Time, ThetaM);
hold on;
plot(Time, theta_ref*ones(numel(1,Time)), 'r--');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angle [rad]');
title(sprintf('Pendulum Angle (settling: %.2f s)', theta_settling_time));

% Pendulum angular velocity
subplot(4,2,7);
plot(Time, ThetaDotM);
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
title('Pendulum Angular Velocity');

% RIGHT SIDE

% Control input - Velocity command and Acceleration
subplot(4,2,2);
yyaxis left
plot(Time, Vcmd);
ylabel('Velocity Command [m/s]');
ylim([min(Vcmd)-0.1, max(Vcmd)+0.1]);
yyaxis right
plot(Time, Accel, 'r-');
ylabel('Acceleration [m/s²]');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
title(sprintf('Control Inputs (V_{cmd} max: %.2f m/s, A max: %.2f m/s²)', max(abs(Vcmd)), max(abs(Accel))));

% Cart animation (spans 3 rows)
subplot(4,2,[4 6 8]);
for k=1:10:length(Time)
    PendCartDraw([PosM(k) ThetaM(k)], Time(k), 'Response');
end

sgtitle('LQR Control Response - Non-Linear Simulation', 'FontSize', 14, 'FontWeight', 'bold');