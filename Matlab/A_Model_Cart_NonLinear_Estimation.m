%% CART MODEL VALIDATION - NON-LINEAR VS LINEAR VS EXPERIMENTAL
% Renan Duarte - Modified from estimation script
% Compares non-linear rate limiter model, linear model, and experimental data

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

%% EXPERIMENTAL DATA

% Import data
try    
    Exp = readtable('CartPend_Steps_Vel2.xlsx', 'Sheet', 1);
catch
    fprintf ("Experimental data not found. Aborting \n\n");    
    return;
end 

% Get data - New format
Time = Exp.Real_Time;
VelCmd = Exp.Target_Vel;
PWM = Exp.PWM_Freq;
Pos = Exp.Cart_Pos_Filt;
PosDot = Exp.Cart_Vel_Filt;
Theta = Exp.Pendulum_Pos_Filt;
ThetaDot = Exp.Pendulum_Vel_Filt;

% Adjust pendulum angle
Theta = pi - Theta;
Theta = wrapToPi(Theta) + pi;
ThetaDot = -ThetaDot;

% Get number of samples
N = length(Pos);

% Calculate average sample time
Ts = mean(diff(Time));

% Recreate uniform time vector
Time = (0:N-1)' * Ts;

fprintf('DATASET INFO (before trimming) \n\n');

fprintf('Sample time: %.6f s (%.1f Hz)\n', Ts, 1/Ts);
fprintf('Data duration: %.2f s\n', Time(end));
fprintf('Number of samples: %d\n\n', length(Time));

% Calculate derivatives for validation
PosDotCalc = gradient(Pos(:)) ./ gradient(Time(:)); 
AccelCalc = gradient(PosDotCalc(:)) ./ gradient(Time(:)); 
ThetaDotCalc = gradient(Theta(:)) ./ gradient(Time(:));

% Clear variables
clear -regexp ^Exp;

%% PLOTS - EXPERIMENTAL DATA

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% Cart Position
subplot(321);
plot(Time, Pos, 'DisplayName', 'Measured');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title('Cart Position');
legend;

% Cart Velocity
subplot(323);
plot(Time, PosDot, 'DisplayName', 'Measured');
hold on;
plot(Time, PosDotCalc, 'DisplayName', 'Calculated (gradient)');
plot(Time, VelCmd, 'r--', 'DisplayName', 'Command');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Cart Velocity');
legend;

% Pendulum Angle (corrected)
subplot(322);
plot(Time, Theta, 'DisplayName', 'Corrected (pi = down)');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angle [rad]');
title('Pendulum Angle');
legend;

% Pendulum Angular Velocity
subplot(324);
plot(Time, ThetaDot, 'DisplayName', 'Measured');
hold on;
plot(Time, ThetaDotCalc, 'DisplayName', 'Calculated (gradient)');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
title('Pendulum Angular Velocity');
legend;

% Cart Acceleration
subplot(325);
plot(Time, AccelCalc, 'DisplayName', 'Calculated');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Acceleration [m/s²]');
title('Cart Acceleration');
legend;

% PWM Frequency
subplot(326);
plot(Time, PWM, 'DisplayName', 'PWM');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('PWM Frequency [Hz]');
title('Motor PWM');
legend;

sgtitle('Experimental Data Analysis');

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
fprintf('Initial conditions: Pos = %.4f m, Vel = %.6f m/s\n\n', ...
    Pos(1), PosDot(1));

%% AUTOMATIC PARAMETER TUNING

fprintf('AUTOMATIC PARAMETER TUNING \n\n');

% Lower and upper bounds
LowerBound = [5, 0.005, 0.01, 1];   % [amax_min, Tr_min, Kv_min, Kp_min]
UpperBound = [10, 0.1, 1, 1];     % [amax_max, Tr_max, Kv_max, Kp_max]

% Initial parameter values for optimization (starting point)
Param0 = LowerBound;

% Method: fmincon with SQP
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'sqp', ...
    'MaxIterations', 200, ...
    'MaxFunctionEvaluations', 600, ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-8, ...
    'FiniteDifferenceStepSize', 1e-6);

costFunction = @(params) computeCost(params, Time, VelCmd, Pos, PosDot);
[params_opt, fval] = fmincon(costFunction, Param0, ...
    [], [], [], [], LowerBound, UpperBound, [], options);

fprintf('Cost: %.8f, Params: [amax=%.4f, Tr=%.6f, Kv=%.4f, Kp=%.4f]\n\n', ...
    fval, params_opt(1), params_opt(2), params_opt(3), params_opt(4));

amax = params_opt(1);
Tr = params_opt(2);
Kv = params_opt(3);
Kp = params_opt(4);

%% CHOOSE PARAMETERS

% Non-linear model parameters
% amax = 7.59963135512758;
% Tr = 1/80;
% Kv = 1;
% Kp = 1;

% Linear model parameters
tau_lin = Tr;
Kv_lin = Kv;
Kp_lin = Kp;

fprintf('FINAL MODEL PARAMETERS \n\n');
fprintf('Non-linear (Rate Limiter):\n');
fprintf('  amax = %.4f m/s^2\n', amax);
fprintf('  Tr = %.6f s\n', Tr);
fprintf('  Kv = %.4f\n', Kv);
fprintf('  Kp = %.4f\n', Kp);
fprintf('\nLinear (1st order):\n');
fprintf('  tau = %.6f s\n', tau_lin);
fprintf('  Kv = %.4f\n', Kv_lin);
fprintf('  Kp = %.4f\n\n', Kp_lin);

%% SIMULATE NON-LINEAR MODEL

Params_NL = [amax, Tr, Kv, Kp];
State0_NL = [Pos(1); PosDot(1)];

[~, States_NL] = ode45(@(SolverTime, State)CartNonLinearModel(SolverTime, State, Params_NL, VelCmd, Time), Time, State0_NL);

Pos_NL = States_NL(:, 1);
PosDot_NL = States_NL(:, 2);

%% SIMULATE LINEAR MODEL

% State-space representation
A_lin = [0,         Kp_lin;
         0,    -1/tau_lin];

B_lin = [0;
         Kv_lin/tau_lin];

C_lin = eye(2);
D_lin = zeros(2,1);

sys_lin = ss(A_lin, B_lin, C_lin, D_lin);

% Simulate
x0_lin = [Pos(1); PosDot(1)];
[States_Lin, ~] = lsim(sys_lin, VelCmd, Time, x0_lin);

Pos_Lin = States_Lin(:, 1);
PosDot_Lin = States_Lin(:, 2);

%% CALCULATE ERRORS

% Non-linear model errors
pos_error_NL = Pos - Pos_NL;
vel_error_NL = PosDot - PosDot_NL;

pos_rmse_NL = sqrt(mean(pos_error_NL.^2));
vel_rmse_NL = sqrt(mean(vel_error_NL.^2));

pos_fit_NL = 100 * (1 - pos_rmse_NL/std(Pos));
vel_fit_NL = 100 * (1 - vel_rmse_NL/std(PosDot));

% Linear model errors
pos_error_Lin = Pos - Pos_Lin;
vel_error_Lin = PosDot - PosDot_Lin;

pos_rmse_Lin = sqrt(mean(pos_error_Lin.^2));
vel_rmse_Lin = sqrt(mean(vel_error_Lin.^2));

pos_fit_Lin = 100 * (1 - pos_rmse_Lin/std(Pos));
vel_fit_Lin = 100 * (1 - vel_rmse_Lin/std(PosDot));

%% PRINT RESULTS

fprintf('VALIDATION RESULTS\n\n');

fprintf('NON-LINEAR MODEL (Rate Limiter):\n');
fprintf('  Position RMSE: %.6f m\n', pos_rmse_NL);
fprintf('  Position FIT:  %.2f %%\n', pos_fit_NL);
fprintf('  Velocity RMSE: %.6f m/s\n', vel_rmse_NL);
fprintf('  Velocity FIT:  %.2f %%\n\n', vel_fit_NL);

fprintf('LINEAR MODEL (1st Order):\n');
fprintf('  Position RMSE: %.6f m\n', pos_rmse_Lin);
fprintf('  Position FIT:  %.2f %%\n', pos_fit_Lin);
fprintf('  Velocity RMSE: %.6f m/s\n', vel_rmse_Lin);
fprintf('  Velocity FIT:  %.2f %%\n\n', vel_fit_Lin);

fprintf('COMPARISON:\n');
fprintf('  Position improvement (NL vs Lin): %.2f %%\n', ...
    100*(pos_rmse_Lin - pos_rmse_NL)/pos_rmse_Lin);
fprintf('  Velocity improvement (NL vs Lin): %.2f %%\n\n', ...
    100*(vel_rmse_Lin - vel_rmse_NL)/vel_rmse_Lin);

%% PLOTS - MODEL COMPARISON

hFig2 = figure(2);
set(hFig2, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);

% Position comparison
subplot(221);
plot(Time, Pos, 'k', 'DisplayName', 'Experimental');
hold on;
plot(Time, Pos_NL, 'b--', 'DisplayName', 'Non-Linear');
plot(Time, Pos_Lin, 'r:', 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title(sprintf('Position (FIT: NL=%.2f%%, Lin=%.2f%%)', pos_fit_NL, pos_fit_Lin));
legend('Location', 'best');

% Velocity comparison
subplot(222);
plot(Time, PosDot, 'k', 'DisplayName', 'Experimental');
hold on;
plot(Time, PosDot_NL, 'b--','DisplayName', 'Non-Linear');
plot(Time, PosDot_Lin, 'r:', 'DisplayName', 'Linear');
plot(Time, VelCmd, 'g--', 'DisplayName', 'Command');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title(sprintf('Velocity (FIT: NL=%.2f%%, Lin=%.2f%%)', vel_fit_NL, vel_fit_Lin));
legend('Location', 'best');

% Position errors
subplot(223);
plot(Time, pos_error_NL, 'b-', 'DisplayName', 'Non-Linear');
hold on;
plot(Time, pos_error_Lin, 'r-', 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [m]');
title(sprintf('Position Error (RMSE: NL=%.6f, Lin=%.6f)', pos_rmse_NL, pos_rmse_Lin));
legend('Location', 'best');

% Velocity errors
subplot(224);
plot(Time, vel_error_NL, 'b-', 'DisplayName', 'Non-Linear');
hold on;
plot(Time, vel_error_Lin, 'r-', 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [m/s]');
title(sprintf('Velocity Error (RMSE: NL=%.6f, Lin=%.6f)', vel_rmse_NL, vel_rmse_Lin));
legend('Location', 'best');

sgtitle('Model Validation: Non-Linear vs Linear vs Experimental');

%% SAVE CART OBJECT

% Dialog popup - Save results
OptSave = SavePopUp;

% If the user chose to save the results, save the optimized parameters
if OptSave    
    Cart.Amax = amax;
    Cart.Tr = Tr;
    Cart.Kv = Kv;
    Cart.Kp = Kp;
    
    % Delete the old results file if it exists and save the new one
    if exist('./Results/Cart.mat', 'file')
        delete('./Results/Cart.mat');
    end
    save('./Results/Cart.mat', 'Cart');    
end

Time_Duration = toc;
fprintf ("\nCalculations took %.2f seconds\n\n", Time_Duration);

%% COST FUNCTION FOR OPTIMIZATION

function cost = computeCost(params, Time, VelCmd, Pos_exp, PosDot_exp)
    % Extract parameters
    amax = params(1);
    Tr = params(2);
    Kv = params(3);
    Kp = params(4);
    
    % Check for invalid parameters
    if amax <= 0 || Tr <= 0 || Kv <= 0 || Kp <= 0
        cost = 1e10;
        return;
    end
    
    % Simulate non-linear model
    Params_NL = [amax, Tr, Kv, Kp];
    State0_NL = [Pos_exp(1); PosDot_exp(1)];
    
    try
        % Use tighter tolerances
        opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.01);
        [~, States_NL] = ode45(@(SolverTime, State)CartNonLinearModel(SolverTime, State, Params_NL, VelCmd, Time), ...
            Time, State0_NL, opts);
        
        Pos_NL = States_NL(:, 1);
        PosDot_NL = States_NL(:, 2);
        
        % Check for NaN or Inf
        if any(isnan(Pos_NL)) || any(isinf(Pos_NL)) || any(isnan(PosDot_NL)) || any(isinf(PosDot_NL))
            cost = 1e10;
            return;
        end
        
        % Calculate errors with different metrics
        
        % 1. RMSE (normalized)
        rmse_pos = sqrt(mean((Pos_exp - Pos_NL).^2)) / std(Pos_exp);
        rmse_vel = sqrt(mean((PosDot_exp - PosDot_NL).^2)) / std(PosDot_exp);
        
        % 2. Max error (to penalize large deviations)
        max_pos_error = max(abs(Pos_exp - Pos_NL)) / std(Pos_exp);
        max_vel_error = max(abs(PosDot_exp - PosDot_NL)) / std(PosDot_exp);
        
        % 3. FIT percentage (inverted to minimize)
        fit_pos = 1 - (1 - sqrt(mean((Pos_exp - Pos_NL).^2))/std(Pos_exp));
        fit_vel = 1 - (1 - sqrt(mean((PosDot_exp - PosDot_NL).^2))/std(PosDot_exp));
        
        % Combined cost with multiple criteria
        w_rmse_pos = 0;
        w_rmse_vel = 0;
        w_max_pos = 0.0;
        w_max_vel = 0.0;
        w_fit_pos = 1;
        w_fit_vel = 0.5;
        
        cost = w_rmse_pos * rmse_pos + ...
               w_rmse_vel * rmse_vel + ...
               w_max_pos * max_pos_error + ...
               w_max_vel * max_vel_error + ...
               w_fit_pos * fit_pos + ...
               w_fit_vel * fit_vel;
        
    catch ME
        % If simulation fails, return large cost
        fprintf('Simulation failed: %s\n', ME.message);
        cost = 1e10;
    end
end