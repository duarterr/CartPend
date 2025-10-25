%% CART CONTROL - STATE FEEDBACK CONTROL
% Simplified version - Cart only
% Modified: 2025

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

%% PENDULUM NON LINEAR MODEL

% Import data
try
    load('./Results/Pendulum.mat');    
catch
    fprintf ("Pendumum data not found. Aborting \n\n");    
    return;
end

m = Pendulum.m;
l = Pendulum.l;
kd = Pendulum.kd;
kdr = Pendulum.kdr;
kc = Pendulum.kc;

%% CART MODEL

Ac = [0 1; 0 0];
Bc = [0 1]';
Cc = [1 0];
Dc = 0;

%% CART MODEL - DISCRETE

% Sample time
Ts = 1/200;

% Discretization
sys_c = ss(Ac, Bc, Cc, Dc);
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

%% POLE PLACEMENT

% Option 1: First-order (no overshoot) - set oscillatory = false
% Option 2: Second-order oscillatory - set oscillatory = true
oscillatory = false;  % Change to false for first-order response

if oscillatory
    % OSCILLATORY RESPONSE (2nd order underdamped)
    
    % Desired specifications
    ts_desired = 5;      % Settling time
    zeta = 0.2;            % Damping ratio (0 < zeta < 1)
                           % zeta = 0.3 → more oscillation
                           % zeta = 0.7 → less oscillation
    
    % Calculate natural frequency from settling time
    % For 2nd order: ts ≈ 4/(zeta*wn)
    wn = 4 / (zeta * ts_desired);
    
    % Overshoot calculation
    Mp = exp(-pi*zeta/sqrt(1-zeta^2)) * 100;
    
    fprintf('Desired specifications:\n');
    fprintf('Settling time: %.3f s\n', ts_desired);
    fprintf('Damping ratio (zeta): %.3f\n', zeta);
    fprintf('Natural frequency (wn): %.3f rad/s\n', wn);
    fprintf('Expected overshoot: %.1f %%\n\n', Mp);
    
    % Complex conjugate poles: s = -zeta*wn ± j*wn*sqrt(1-zeta^2)
    real_part = -zeta * wn;
    imag_part = wn * sqrt(1 - zeta^2);
    
    p1 = real_part + 1j*imag_part;
    p2 = real_part - 1j*imag_part;

else
    % Desired specifications for first-order response
    % Settling time (using 2% criterion: ts = 4/wn for 1st order)
    ts_desired = 2; % s
    wn = 4 / ts_desired;  % Natural frequency (cutoff frequency) - rad/s

    fprintf('Desired specifications:\n');
    fprintf('Settling time: %.3f s\n', ts_desired);
    fprintf('Cutoff frequency (wn): %.3f rad/s\n\n', wn);

    % Continuous-time poles
    % Dominant pole at -wn (first-order behavior)
    % Second pole at -10*wn (far away, minimal influence)
    p1 = -wn;
    p2 = -10 * wn;
end

% Convert continuous poles to discrete using z = exp(s*Ts)
p1z = exp(p1 * Ts);
p2z = exp(p2 * Ts);

desired_poles_d = [p1z; p2z];

fprintf('Desired continuous poles (s):\n');
fprintf('p1 = %.3f + j%.3f\n', real(p1), imag(p1));
fprintf('p2 = %.3f - j%.3f\n\n', real(p2), imag(p2));

fprintf('Desired discrete poles (z):\n');
fprintf('p1 = %.6f + j%.6f  (|z| = %.6f, angle = %.3f rad)\n', ...
        real(p1z), imag(p1z), abs(p1z), angle(p1z));
fprintf('p2 = %.6f - j%.6f  (|z| = %.6f, angle = %.3f rad)\n\n', ...
        real(p2z), imag(p2z), abs(p2z), angle(p2z));
        
% State feedback design in discrete time
K = acker(Ad, Bd, desired_poles_d);

fprintf('Controller gains:\n');
fprintf('K = [%.6f, %.6f]\n', K(1), K(2));

%% POLES LOCATIONS

p = eig(Ad);
fprintf('\nOpen loop poles (z):\n');
display(p);

p = eig(Ad - Bd*K);
fprintf('\nClosed loop poles (z):\n');
display(p);

%% CLOSED-LOOP SYSTEM 

% Calculate feedforward gain Kr explicitly for zero steady-state error
% For step reference: Kr = 1 / (C * inv(I - A + B*K) * B)
% Or using the formula: Kr = inv(Ccl * inv(eye(size(Acl)) - Acl) * Bcl_temp)
% where Bcl_temp = Bd (before applying Kr)

Acl = Ad - Bd*K;

% Calculate Kr for zero steady-state error to step input
% Method: Kr = 1 / (Cd * inv(I - Acl) * Bd)
Kr = 1 / (Cd * inv(eye(size(Acl)) - Acl) * Bd);

fprintf('\nFeedforward gain (Kr):\n');
fprintf('Kr = %.6f\n\n', Kr);

%% PERFORMANCE EVALUATION

% Parameters for nonlinear model
ParamsOpt = [m l kd kdr kc];

% Time vector
Time = 0:Ts:3;
N = length(Time);

% Step reference of 0.1m
ref = 0.1 * ones(N, 1);

% Initial state
States = zeros(N, 4);
States(1,:) = [0; 0; pi; 0];

% Control input
Accel = zeros(N, 1);

% Simulate using NONLINEAR model with control from linear design
for i = 1:N-1
    % State feedback control using only cart states [pos; vel]
    Accel(i) = -K * States(i, 1:2)' + Kr * ref(i);
    
    % Simulate NONLINEAR model with CartPendModel
    tspan = [Time(i), Time(i+1)];
    [~, y_temp] = ode45(@(t, y) CartPendModel(t, y, ParamsOpt, Accel(i), Time(i)), ...
                        tspan, States(i,:)');
    
    States(i+1,:) = y_temp(end,:);
end

% Extract states
PosM = States(:,1);
VelM = States(:,2);
ThetaM = States(:,3);
ThetaDotM = States(:,4);

%% PERFORMANCE METRICS

% Settling time (2% criterion)
final_value = PosM(end);
tol = 0.02 * abs(final_value);
settling_idx = find(abs(PosM - final_value) > tol, 1, 'last');
if isempty(settling_idx)
    settling_time = 0;
else
    settling_time = Time(settling_idx);
end

% Overshoot
overshoot = (max(PosM) - final_value) / final_value * 100;

% Rise time (10% to 90%)
idx_10 = find(PosM >= 0.1*final_value, 1);
idx_90 = find(PosM >= 0.9*final_value, 1);
rise_time = Time(idx_90) - Time(idx_10);

fprintf('Performance Metrics:\n');
fprintf('Settling time (2%%): %.3f s\n', settling_time);
fprintf('Overshoot: %.2f %%\n', overshoot);
fprintf('Rise time: %.3f s\n', rise_time);
fprintf('Steady-state error: %.6f m\n\n', ref(end) - PosM(end));

% Control effort
max_control = max(abs(Accel));
fprintf('Maximum control: %.3f m/s²\n\n', max_control);

Time_Duration = toc;
fprintf('Calculations took %.2f seconds\n', Time_Duration);

%% PLOTS

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition', [0.1 0.1 0.8 0.8]);
clf(1);

% Position tracking
subplot(321);
plot(Time, ref, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
hold on;
plot(Time, PosM, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Position');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title('Cart Position Tracking');
legend('Location', 'best');

% Velocity
subplot(323);
plot(Time, VelM, 'b-', 'LineWidth', 1.5);
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Cart Velocity');

% Control input
subplot(325);
plot(Time, Accel, 'b-', 'LineWidth', 1.5);
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Acceleration [m/s²]');
title('Control Input');

% Add overall title
sgtitle('Cart State Feedback Control - Position reference');

% State portrait
subplot(322);
plot(PosM, VelM, 'b-', 'LineWidth', 1.5);
hold on;
plot(PosM(1), VelM(1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Initial');
plot(PosM(end), VelM(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Final');
plot(0, 0, 'k*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Equilibrium');
grid on;
xlabel('Position [m]');
ylabel('Velocity [m/s]');
title('State Portrait');
legend('Trajectory', 'Initial State', 'Final State', 'Equilibrium');

% Cart animation
subplot(3,2,[4 6]);
for k=1:10:length(Time)
    PendCartDraw([PosM(k) ThetaM(k)], Time(k), 'Response');
    %pause(Time(k+1)-Time(k));
end