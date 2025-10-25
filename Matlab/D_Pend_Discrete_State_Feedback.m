%% PENDULUM CONTROL - STATE FEEDBACK CONTROL
% Simplified version - Pendulum only (stabilization at upright position)
% Created: 2025

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

%% PENDULUM MODEL - LINEARIZED AROUND UPRIGHT POSITION (θ = 0)

g = 9.81;  % gravity [m/s^2]

% Moment of inertia
I = m*l^2;

% Linearized state-space coefficients
a = m*g*l/I;
b = -kd/I;
c = -m*l/I;

% State-space model: x = [theta; theta_dot]
% Input: torque or horizontal acceleration at pivot
Ap = [0 1; 
      a b];
Bp = [0; 
      c];  % Input as horizontal acceleration at pivot
Cp = [1 0];
Dp = 0;

%% PENDULUM MODEL - DISCRETE

% Sample time
Ts = 1/200;  % 200 Hz

% Discretization using zero-order hold
sys_c = ss(Ap, Bp, Cp, Dp);
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

%% POLE PLACEMENT

ts_desired = 0.4;   % Settling time [s]
zeta = 0.8;         % Damping ratio

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

% State feedback design in discrete time using Ackermann's formula
K = acker(Ad, Bd, desired_poles_d);

fprintf('Controller gains:\n');
fprintf('K = [%.6f, %.6f]\n\n', K(1), K(2));

%% POLES LOCATIONS

p = eig(Ad);
fprintf('Open loop poles (z):\n');
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

%% PERFORMANCE EVALUATION - REGULATION (STABILIZATION)

% Parameters for nonlinear model
ParamsOpt = [m l kd kdr kc];

% Time vector
Time = 0:Ts:3;
N = length(Time);

% Initial state
States = zeros(N, 4);
States(1,:) = [0; 0; 0.1; 0];

% Control input
Accel = zeros(N, 1);

% Simulate using NONLINEAR model with control from linear design
for i = 1:N-1
    % State feedback control using only pend states [theta; thetadot]
    Accel(i) = -K * States(i, 3:4)'; % No reference (regulation)
    
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
final_value = 0;  % Target is upright position
tol = 0.02 * abs(ThetaM(1));
settling_idx = find(abs(ThetaM) > tol, 1, 'last');
if isempty(settling_idx)
    settling_time = 0;
else
    settling_time = Time(settling_idx);
end

% Maximum deviation (overshoot from initial)
max_deviation = max(abs(ThetaM));

fprintf('Performance Metrics:\n');
fprintf('Initial angle: %.2f rad\n', ThetaM(1));
fprintf('Settling time (2%%): %.3f s\n', settling_time);
fprintf('Maximum deviation: %.2f rad\n', max_deviation);
fprintf('Final error: %.4f rad\n\n', abs(ThetaM(end)));

% Control effort
max_control = max(abs(Accel));
fprintf('Maximum control: %.3f m/s²\n\n', max_control);

Time_Duration = toc;
fprintf('Calculations took %.2f seconds\n\n', Time_Duration);

%% PLOTS

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition', [0.1 0.1 0.8 0.8]);
clf(1);

% Angle (degrees)
subplot(321);
plot(Time, ThetaM, 'b-', 'LineWidth', 1.5);
hold on;
plot(Time, zeros(size(Time)), 'r--', 'LineWidth', 1, 'DisplayName', 'Target');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angle [degrees]');
title('Pendulum Angle (from upright)');
legend('Angle', 'Target', 'Location', 'best');

% Angular velocity
subplot(323);
plot(Time, ThetaDotM, 'b-', 'LineWidth', 1.5);
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
title('Pendulum Angular Velocity');

% Control input
subplot(325);
plot(Time, Accel, 'b-', 'LineWidth', 1.5);
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Control Input [m/s²]');
title('Control Input (Horizontal Acceleration)');

% Add overall title
sgtitle('Pendulum State Feedback Control - Stabilization at Upright Position');

% State portrait
subplot(322);
plot(ThetaM, ThetaDotM, 'b-', 'LineWidth', 1.5);
hold on;
plot(ThetaM(1), ThetaDotM(1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Initial');
plot(ThetaM(end), ThetaDotM(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Final');
plot(0, 0, 'k*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Equilibrium');
grid on;
xlabel('Angle [rad]');
ylabel('Angular Velocity [rad/s]');
title('State Portrait');
legend('Trajectory', 'Initial State', 'Final State', 'Equilibrium');

% Cart animation
subplot(3,2,[4 6]);
for k=1:10:length(Time)
    PendCartDraw([PosM(k) ThetaM(k)], Time(k), 'Response');
    %pause(Time(k+1)-Time(k));
end
