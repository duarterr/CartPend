%% CART-PENDULUM COMPLETE SYSTEM VALIDATION
% Combined validation of cart and pendulum models
% Compares non-linear and linear models against experimental data
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
    fprintf('Cart parameters not found. Aborting \n');
    return;
end

% Load pendulum parameters
try
    load('./Results/Pendulum.mat', 'Pendulum', 'PendulumLinear');
    fprintf('Pendulum parameters loaded successfully\n\n');
catch
    fprintf('Pendulum parameters not found. Aborting\n');
    return;
end

% Display loaded parameters
fprintf('LOADED PARAMETERS\n\n');
fprintf('Cart (Non-Linear):\n');
fprintf('  amax = %.4f m/s^2\n', Cart.Amax);
fprintf('  Tr   = %.6f s\n', Cart.Tr);
fprintf('  Kv   = %.4f\n', Cart.Kv);
fprintf('  Kp   = %.4f\n\n', Cart.Kp);

fprintf('Pendulum (Non-Linear):\n');
fprintf('  m    = %.6f kg\n', Pendulum.m);
fprintf('  l    = %.6f m\n', Pendulum.l);
fprintf('  kd   = %.6e Ns/m\n', Pendulum.kd);
fprintf('  kdr  = %.6e Ns^2/m^2\n', Pendulum.kdr);
fprintf('  kc   = %.6e N\n\n', Pendulum.kc);

fprintf('Pendulum (Linear):\n');
fprintf('  m    = %.6f kg\n', PendulumLinear.m);
fprintf('  l    = %.6f m\n', PendulumLinear.l);
fprintf('  kd   = %.6e Ns/m\n\n', PendulumLinear.kd);

%% EXPERIMENTAL DATA

% Import data
try    
    Exp = readtable('CartPend_Steps_Vel2.xlsx', 'Sheet', 1);
    fprintf('Experimental data loaded successfully\n');
catch
    fprintf('Experimental data not found. Aborting\n\n');    
    return;
end 

% Get data - New format
Time = Exp.Real_Time;
VelCmd = Exp.Target_Vel;
PWM = Exp.PWM_Freq;
Pos = Exp.Cart_Pos;
PosDot = Exp.Cart_Vel;
Theta = Exp.Pendulum_Pos;
ThetaDot = Exp.Pendulum_Vel;

% Adjust pendulum angle (convert to convention: pi = down)
ThetaEq = Theta(1);
Theta = wrapToPi(Theta - ThetaEq) + pi;  

% Get number of samples
N = length(Pos);

% Calculate average sample time
Ts = mean(diff(Time));

% Recreate uniform time vector
Time = (0:N-1)' * Ts;

fprintf('\nDATASET INFO\n\n');
fprintf('Sample time: %.6f s (%.1f Hz)\n', Ts, 1/Ts);
fprintf('Data duration: %.2f s\n', Time(end));
fprintf('Number of samples: %d\n', length(Time));
fprintf('Initial conditions:\n');
fprintf('  Cart: p = %.4f m, v = %.4f m/s\n', Pos(1), PosDot(1));
fprintf('  Pendulum: theta = %.4f rad, theta_dot = %.4f rad/s\n\n', Theta(1), ThetaDot(1));

% Calculate derivatives for validation
PosDotCalc = gradient(Pos(:)) ./ gradient(Time(:)); 
AccelCalc = gradient(PosDotCalc(:)) ./ gradient(Time(:)); 
ThetaDotCalc = gradient(Theta(:)) ./ gradient(Time(:));

%% REDUCE DATA SIZE

% Ignore data after this time
Stop_Time = 100;

Time = Time(Time < Stop_Time);
VelCmd = VelCmd(Time < Stop_Time);
PWM = PWM(Time < Stop_Time);
Pos = Pos(Time < Stop_Time);
PosDot = PosDot(Time < Stop_Time);
Theta = Theta(Time < Stop_Time);
ThetaDot = ThetaDot(Time < Stop_Time);
PosDotCalc = PosDotCalc(Time < Stop_Time);
AccelCalc = AccelCalc(Time < Stop_Time);
ThetaDotCalc = ThetaDotCalc(Time < Stop_Time);

fprintf('DATASET INFO (after trimming)\n\n');

fprintf('Sample time: %.6f s (%.1f Hz)\n', Ts, 1/Ts);
fprintf('Data duration: %.2f s\n', Time(end));
fprintf('Number of samples: %d\n', length(Time));
fprintf('Initial conditions: Pos = %.4f m, Vel = %.6f m/s, theta = %.4f rad, theta_dot = %.6f rad/s\n\n', ...
    Pos(1), PosDot(1), Theta(1), ThetaDot(1));

%% SIMULATE NON-LINEAR COMPLETE MODEL

fprintf('SIMULATING NON-LINEAR MODEL...\n');

% Combined parameters vector for complete model
Params_NL = [Cart.Amax, Cart.Tr, Cart.Kv, Cart.Kp, ...
             Pendulum.m, Pendulum.l, Pendulum.kd, Pendulum.kdr, Pendulum.kc];

% Initial state vector: [p, v, theta, theta_dot]
State0_NL = [Pos(1); PosDot(1); Theta(1); ThetaDot(1)];

% Solve ODE
opts_NL = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', Ts);
[~, States_NL] = ode45(@(SolverTime, State)CartPendNonLinearModel(SolverTime, State, Params_NL, VelCmd, Time), ...
                        Time, State0_NL, opts_NL);

% Extract states
Pos_NL = States_NL(:, 1);
PosDot_NL = States_NL(:, 2);
Theta_NL = States_NL(:, 3);
ThetaDot_NL = States_NL(:, 4);

fprintf('Non-linear simulation completed\n\n');

%% BUILD LINEAR MODEL

fprintf('BUILDING LINEAR MODEL...\n');

% Constants
g = 9.81;

% Cart linear parameters
tau = Cart.Tr;
Kv = Cart.Kv;
Kp = Cart.Kp;

% Pendulum linear parameters
m = PendulumLinear.m;
l = PendulumLinear.l;
kd = PendulumLinear.kd;
I = m * l^2;

% Determine linearization point based on pendulum configuration
PendulumUp = false;
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

% Create state-space system
sys_lin = ss(A, B, C, D);

fprintf('Linear state-space model created\n');
fprintf('System order: %d states\n', size(A, 1));
fprintf('Number of inputs: %d\n', size(B, 2));
fprintf('Number of outputs: %d\n\n', size(C, 1));

%% SIMULATE LINEAR MODEL

fprintf('SIMULATING LINEAR MODEL...\n');

% Initial state in phi coordinates: [p, v, phi, phi_dot]'
phi0 = Theta(1) - pi;
phi_dot0 = ThetaDot(1);
State0_Lin = [Pos(1); PosDot(1); phi0; phi_dot0];

% Simulate
[States_Lin, ~] = lsim(sys_lin, VelCmd, Time, State0_Lin);

% Extract states (convert phi back to theta)
Pos_Lin = States_Lin(:, 1);
PosDot_Lin = States_Lin(:, 2);
Phi_Lin = States_Lin(:, 3);
PhiDot_Lin = States_Lin(:, 4);
Theta_Lin = Phi_Lin + pi;
ThetaDot_Lin = PhiDot_Lin;

fprintf('Linear simulation completed\n\n');

%% CALCULATE ERRORS

fprintf('CALCULATING ERRORS...\n\n');

% Non-linear model errors
pos_error_NL = Pos - Pos_NL;
vel_error_NL = PosDot - PosDot_NL;
theta_error_NL = Theta - Theta_NL;
thetadot_error_NL = ThetaDot - ThetaDot_NL;

pos_rmse_NL = sqrt(mean(pos_error_NL.^2));
vel_rmse_NL = sqrt(mean(vel_error_NL.^2));
theta_rmse_NL = sqrt(mean(theta_error_NL.^2));
thetadot_rmse_NL = sqrt(mean(thetadot_error_NL.^2));

pos_fit_NL = 100 * (1 - pos_rmse_NL/std(Pos));
vel_fit_NL = 100 * (1 - vel_rmse_NL/std(PosDot));
theta_fit_NL = 100 * (1 - theta_rmse_NL/std(Theta));
thetadot_fit_NL = 100 * (1 - thetadot_rmse_NL/std(ThetaDot));

% Linear model errors
pos_error_Lin = Pos - Pos_Lin;
vel_error_Lin = PosDot - PosDot_Lin;
theta_error_Lin = Theta - Theta_Lin;
thetadot_error_Lin = ThetaDot - ThetaDot_Lin;

pos_rmse_Lin = sqrt(mean(pos_error_Lin.^2));
vel_rmse_Lin = sqrt(mean(vel_error_Lin.^2));
theta_rmse_Lin = sqrt(mean(theta_error_Lin.^2));
thetadot_rmse_Lin = sqrt(mean(thetadot_error_Lin.^2));

pos_fit_Lin = 100 * (1 - pos_rmse_Lin/std(Pos));
vel_fit_Lin = 100 * (1 - vel_rmse_Lin/std(PosDot));
theta_fit_Lin = 100 * (1 - theta_rmse_Lin/std(Theta));
thetadot_fit_Lin = 100 * (1 - thetadot_rmse_Lin/std(ThetaDot));

%% PRINT VALIDATION RESULTS

fprintf('VALIDATION RESULTS SUMMARY\n');

fprintf('NON-LINEAR MODEL:\n');
fprintf('  Cart Position:\n');
fprintf('    RMSE: %.6f m\n', pos_rmse_NL);
fprintf('    FIT:  %.2f %%\n', pos_fit_NL);
fprintf('  Cart Velocity:\n');
fprintf('    RMSE: %.6f m/s\n', vel_rmse_NL);
fprintf('    FIT:  %.2f %%\n', vel_fit_NL);
fprintf('  Pendulum Angle:\n');
fprintf('    RMSE: %.6f rad (%.2f deg)\n', theta_rmse_NL, theta_rmse_NL*180/pi);
fprintf('    FIT:  %.2f %%\n', theta_fit_NL);
fprintf('  Pendulum Angular Velocity:\n');
fprintf('    RMSE: %.6f rad/s\n', thetadot_rmse_NL);
fprintf('    FIT:  %.2f %%\n\n', thetadot_fit_NL);

fprintf('LINEAR MODEL:\n');
fprintf('  Cart Position:\n');
fprintf('    RMSE: %.6f m\n', pos_rmse_Lin);
fprintf('    FIT:  %.2f %%\n', pos_fit_Lin);
fprintf('  Cart Velocity:\n');
fprintf('    RMSE: %.6f m/s\n', vel_rmse_Lin);
fprintf('    FIT:  %.2f %%\n', vel_fit_Lin);
fprintf('  Pendulum Angle:\n');
fprintf('    RMSE: %.6f rad (%.2f deg)\n', theta_rmse_Lin, theta_rmse_Lin*180/pi);
fprintf('    FIT:  %.2f %%\n', theta_fit_Lin);
fprintf('  Pendulum Angular Velocity:\n');
fprintf('    RMSE: %.6f rad/s\n', thetadot_rmse_Lin);
fprintf('    FIT:  %.2f %%\n\n', thetadot_fit_Lin);

fprintf('IMPROVEMENT (Non-Linear vs Linear):\n');
fprintf('  Cart Position:     %+.2f %%\n', 100*(pos_rmse_Lin - pos_rmse_NL)/pos_rmse_Lin);
fprintf('  Cart Velocity:     %+.2f %%\n', 100*(vel_rmse_Lin - vel_rmse_NL)/vel_rmse_Lin);
fprintf('  Pendulum Angle:    %+.2f %%\n', 100*(theta_rmse_Lin - theta_rmse_NL)/theta_rmse_Lin);
fprintf('  Pendulum Velocity: %+.2f %%\n\n', 100*(thetadot_rmse_Lin - thetadot_rmse_NL)/thetadot_rmse_Lin);

%% COMPREHENSIVE COMPARISON PLOTS

hFig2 = figure(1);
set(hFig2, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);

% ===== CART STATES =====

% Cart Position
subplot(4, 2, 1);
plot(Time, Pos, 'k-', 'LineWidth', 2, 'DisplayName', 'Experimental');
hold on;
plot(Time, Pos_NL, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
plot(Time, Pos_Lin, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title(sprintf('Cart Position (FIT: NL=%.1f%%, Lin=%.1f%%)', pos_fit_NL, pos_fit_Lin));
legend('Location', 'best');

% Cart Velocity
subplot(4, 2, 3);
plot(Time, PosDot, 'k-', 'LineWidth', 2, 'DisplayName', 'Experimental');
hold on;
plot(Time, PosDot_NL, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
plot(Time, PosDot_Lin, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Linear');
plot(Time, VelCmd, 'g-.', 'LineWidth', 1, 'DisplayName', 'Command');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title(sprintf('Cart Velocity (FIT: NL=%.1f%%, Lin=%.1f%%)', vel_fit_NL, vel_fit_Lin));
legend('Location', 'best');

% ===== PENDULUM STATES =====

% Pendulum Angle
subplot(4, 2, 2);
plot(Time, Theta, 'k-', 'LineWidth', 2, 'DisplayName', 'Experimental');
hold on;
plot(Time, Theta_NL, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
plot(Time, Theta_Lin, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angle [rad]');
title(sprintf('Pendulum Angle (FIT: NL=%.1f%%, Lin=%.1f%%)', theta_fit_NL, theta_fit_Lin));
legend('Location', 'best');

% Pendulum Angular Velocity
subplot(4, 2, 4);
plot(Time, ThetaDot, 'k-', 'LineWidth', 2, 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaDot_NL, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
plot(Time, ThetaDot_Lin, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
title(sprintf('Pendulum Angular Velocity (FIT: NL=%.1f%%, Lin=%.1f%%)', thetadot_fit_NL, thetadot_fit_Lin));
legend('Location', 'best');

% ===== ERRORS =====

% Cart Position Error
subplot(4, 2, 5);
plot(Time, pos_error_NL*1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
hold on;
plot(Time, pos_error_Lin*1000, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [mm]');
title(sprintf('Cart Position Error (RMSE: NL=%.2f mm, Lin=%.2f mm)', pos_rmse_NL*1000, pos_rmse_Lin*1000));
legend('Location', 'best');

% Cart Velocity Error
subplot(4, 2, 7);
plot(Time, vel_error_NL*1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
hold on;
plot(Time, vel_error_Lin*1000, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [mm/s]');
title(sprintf('Cart Velocity Error (RMSE: NL=%.2f mm/s, Lin=%.2f mm/s)', vel_rmse_NL*1000, vel_rmse_Lin*1000));
legend('Location', 'best');

% Pendulum Angle Error
subplot(4, 2, 6);
plot(Time, theta_error_NL*180/pi, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
hold on;
plot(Time, theta_error_Lin*180/pi, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [deg]');
title(sprintf('Pendulum Angle Error (RMSE: NL=%.2f°, Lin=%.2f°)', theta_rmse_NL*180/pi, theta_rmse_Lin*180/pi));
legend('Location', 'best');

% Pendulum Angular Velocity Error
subplot(4, 2, 8);
plot(Time, thetadot_error_NL*180/pi, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
hold on;
plot(Time, thetadot_error_Lin*180/pi, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [deg/s]');
title(sprintf('Pendulum Angular Velocity Error (RMSE: NL=%.2f°/s, Lin=%.2f°/s)', ...
      thetadot_rmse_NL*180/pi, thetadot_rmse_Lin*180/pi));
legend('Location', 'best');

sgtitle('Complete Cart-Pendulum System Validation', 'FontSize', 16, 'FontWeight', 'bold');

%% SAVE RESULTS

Time_Duration = toc;
fprintf('Validation completed in %.2f seconds \n', Time_Duration);