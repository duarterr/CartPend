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

% Dialog popup - Save results
% OptSave = SavePopUp;

% Start counting time
tic;

%% EXPERIMENTAL DATA

% Import data
try    
    Exp = readtable('CartPend_Steps_Vel.xlsx', 'Sheet', 1);
catch
    fprintf ("Experimental data not found. Aborting \n\n");    
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

% Adjust pendulum angle
Theta = pi - Theta;
Theta = wrapToPi(Theta);
ThetaDot = -ThetaDot;

% Get number of samples
N = length(Pos);

% Calculate average sample time
Ts = mean(diff(Time));

% Recreate uniform time vector
Time = (0:N-1)' * Ts;

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
plot(Time, Theta*180/pi, 'DisplayName', 'Corrected (0° = down)');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angle [degrees]');
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

%% AUTOMATIC PARAMETER TUNING - IMPROVED

fprintf('\n=== STARTING AUTOMATIC PARAMETER TUNING ===\n\n');

% Initial guess for parameters [amax, Tr, Kv]
params_initial = [7.5, 1/60, 1.35];

% Lower and upper bounds
lb = [3.0,  0.005,  0.8];   % [amax_min, Tr_min, Kv_min]
ub = [15.0, 0.1,    2.0];   % [amax_max, Tr_max, Kv_max]

% Try different optimization algorithms
fprintf('Testing different optimization methods...\n\n');

% Method 1: fmincon with SQP
options1 = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'sqp', ...
    'MaxIterations', 200, ...
    'MaxFunctionEvaluations', 600, ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-8, ...
    'FiniteDifferenceStepSize', 1e-6);

fprintf('=== Method 1: fmincon with SQP ===\n');
costFunction = @(params) computeCost(params, Time, VelCmd, Pos, PosDot);
[params_opt1, fval1] = fmincon(costFunction, params_initial, ...
    [], [], [], [], lb, ub, [], options1);

fprintf('Cost: %.8f, Params: [%.4f, %.6f, %.4f]\n\n', fval1, params_opt1);

% Method 2: fmincon with interior-point
options2 = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'interior-point', ...
    'MaxIterations', 200, ...
    'MaxFunctionEvaluations', 600);

fprintf('=== Method 2: fmincon with interior-point ===\n');
[params_opt2, fval2] = fmincon(costFunction, params_initial, ...
    [], [], [], [], lb, ub, [], options2);

fprintf('Cost: %.8f, Params: [%.4f, %.6f, %.4f]\n\n', fval2, params_opt2);

% Method 4: Manual grid search around initial guess (coarse)
fprintf('=== Method 4: Grid Search (coarse) ===\n');
amax_range = linspace(5.0, 10.0, 5);
Tr_range = linspace(0.01, 0.05, 5);
Kv_range = linspace(1.0, 1.7, 5);

best_cost = inf;
best_params = params_initial;

total_iter = length(amax_range) * length(Tr_range) * length(Kv_range);
iter_count = 0;

for amax_val = amax_range
    for Tr_val = Tr_range
        for Kv_val = Kv_range
            iter_count = iter_count + 1;
            params_test = [amax_val, Tr_val, Kv_val];
            cost = costFunction(params_test);
            
            if cost < best_cost
                best_cost = cost;
                best_params = params_test;
                fprintf('Iter %d/%d: New best cost = %.8f, Params = [%.4f, %.6f, %.4f]\n', ...
                    iter_count, total_iter, cost, params_test);
            end
        end
    end
end

params_opt4 = best_params;
fval4 = best_cost;
fprintf('Final Grid Search Cost: %.8f, Params: [%.4f, %.6f, %.4f]\n\n', fval4, params_opt4);

% Select best result
[fval_best, best_method] = min([fval1, fval2, fval4]);
params_all = [params_opt1; params_opt2; params_opt4];
params_optimal = params_all(best_method, :);

method_names = {'fmincon-SQP', 'fmincon-IP', 'patternsearch', 'grid-search'};
fprintf('\n=== BEST METHOD: %s ===\n', method_names{best_method});
fprintf('Best cost: %.8f\n', fval_best);
fprintf('Optimized parameters:\n');
fprintf('  amax = %.4f m/s^2\n', params_optimal(1));
fprintf('  Tr   = %.6f s\n', params_optimal(2));
fprintf('  Kv   = %.4f\n', params_optimal(3));

% Also show manual tuning for comparison
fprintf('\n=== MANUAL TUNING (for comparison) ===\n');
params_manual = [7.5, 1/60, 1.35];
cost_manual = costFunction(params_manual);
fprintf('Manual cost: %.8f\n', cost_manual);
fprintf('Manual params: [%.4f, %.6f, %.4f]\n', params_manual);

if cost_manual < fval_best
    fprintf('\n*** WARNING: Manual tuning is better! Using manual parameters. ***\n');
    params_optimal = params_manual;
    fval_best = cost_manual;
end

%% USE OPTIMIZED PARAMETERS

% Non-linear model parameters
amax = amax_opt;
Tr = Tr_opt;
Kv = Kv_opt;

% Linear model parameters
tau_lin = Tr;
K_lin = Kv;

fprintf('\n=== FINAL MODEL PARAMETERS ===\n');
fprintf('Non-linear (Rate Limiter):\n');
fprintf('  amax = %.4f m/s^2\n', amax);
fprintf('  Tr = %.6f s\n', Tr);
fprintf('  Kv = %.4f\n', Kv);
fprintf('\nLinear (1st order):\n');
fprintf('  tau = %.6f s\n', tau_lin);
fprintf('  K = %.4f\n\n', K_lin);

%% SIMULATE NON-LINEAR MODEL

Params_NL = [amax, Tr, Kv];
State0_NL = [Pos(1); PosDot(1)];

[~, States_NL] = ode45(@(SolverTime, State)CartRateLimiterModel(SolverTime, State, Params_NL, VelCmd, Time), Time, State0_NL);

Pos_NL = States_NL(:, 1);
PosDot_NL = States_NL(:, 2);

%% SIMULATE LINEAR MODEL

% State-space representation
A_lin = [0,         1;
         0,    -1/tau_lin];

B_lin = [0;
         K_lin/tau_lin];

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

fprintf('\n=== VALIDATION RESULTS ===\n\n');

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
subplot(321);
plot(Time, Pos, 'k-', 'LineWidth', 2, 'DisplayName', 'Experimental');
hold on;
plot(Time, Pos_NL, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
plot(Time, Pos_Lin, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title(sprintf('Position (FIT: NL=%.2f%%, Lin=%.2f%%)', pos_fit_NL, pos_fit_Lin));
legend('Location', 'best');

% Velocity comparison
subplot(322);
plot(Time, PosDot, 'k-', 'LineWidth', 2, 'DisplayName', 'Experimental');
hold on;
plot(Time, PosDot_NL, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
plot(Time, PosDot_Lin, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Linear');
plot(Time, VelCmd, 'g--', 'LineWidth', 1, 'DisplayName', 'Command');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title(sprintf('Velocity (FIT: NL=%.2f%%, Lin=%.2f%%)', vel_fit_NL, vel_fit_Lin));
legend('Location', 'best');

% Position errors
subplot(323);
plot(Time, pos_error_NL, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
hold on;
plot(Time, pos_error_Lin, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [m]');
title(sprintf('Position Error (RMSE: NL=%.6f, Lin=%.6f)', pos_rmse_NL, pos_rmse_Lin));
legend('Location', 'best');

% Velocity errors
subplot(324);
plot(Time, vel_error_NL, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
hold on;
plot(Time, vel_error_Lin, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [m/s]');
title(sprintf('Velocity Error (RMSE: NL=%.6f, Lin=%.6f)', vel_rmse_NL, vel_rmse_Lin));
legend('Location', 'best');

% Comparison of models (position)
subplot(325);
plot(Time, Pos_NL, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
hold on;
plot(Time, Pos_Lin, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Linear');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title('Model Comparison: Position');
legend('Location', 'best');

% Comparison of models (velocity)
subplot(326);
plot(Time, PosDot_NL, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
hold on;
plot(Time, PosDot_Lin, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Linear');
plot(Time, VelCmd, 'g--', 'LineWidth', 1, 'DisplayName', 'Command');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Model Comparison: Velocity');
legend('Location', 'best');

sgtitle('Model Validation: Non-Linear vs Linear vs Experimental');

%% PLOTS - ERROR ANALYSIS

% hFig3 = figure(3);
% set(hFig3, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
% 
% % Position error histogram
% subplot(221);
% histogram(pos_error_NL, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Non-Linear');
% hold on;
% histogram(pos_error_Lin, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Linear');
% grid on;
% xlabel('Position Error [m]');
% ylabel('Frequency');
% title('Position Error Distribution');
% legend;
% 
% % Velocity error histogram
% subplot(222);
% histogram(vel_error_NL, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Non-Linear');
% hold on;
% histogram(vel_error_Lin, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'Linear');
% grid on;
% xlabel('Velocity Error [m/s]');
% ylabel('Frequency');
% title('Velocity Error Distribution');
% legend;
% 
% % Absolute position error
% subplot(223);
% plot(Time, abs(pos_error_NL), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
% hold on;
% plot(Time, abs(pos_error_Lin), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear');
% grid on;
% xlim([0 Time(end)]);
% xlabel('Time [s]');
% ylabel('Absolute Error [m]');
% title('Absolute Position Error');
% legend('Location', 'best');
% 
% % Absolute velocity error
% subplot(224);
% plot(Time, abs(vel_error_NL), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Non-Linear');
% hold on;
% plot(Time, abs(vel_error_Lin), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear');
% grid on;
% xlim([0 Time(end)]);
% xlabel('Time [s]');
% ylabel('Absolute Error [m/s]');
% title('Absolute Velocity Error');
% legend('Location', 'best');
% 
% sgtitle('Error Analysis');

%% SAVE RESULTS

if (OptSave)
    % Save figure 1
    if exist('./Results/B0_Experimental_Data.png', 'file')
        delete('./Results/B0_Experimental_Data.png');
    end
    saveas(figure(1), "./Results/B0_Experimental_Data", 'png');
    
    % Save figure 2
    if exist('./Results/B0_Model_Comparison.png', 'file')
        delete('./Results/B0_Model_Comparison.png');
    end
    saveas(figure(2), "./Results/B0_Model_Comparison", 'png');
    
    % Save figure 3
    if exist('./Results/B0_Error_Analysis.png', 'file')
        delete('./Results/B0_Error_Analysis.png');
    end
    saveas(figure(3), "./Results/B0_Error_Analysis", 'png');
    
    % Save results
    Results.amax = amax;
    Results.Tr = Tr;
    Results.tau_lin = tau_lin;
    Results.K_lin = K_lin;
    Results.pos_rmse_NL = pos_rmse_NL;
    Results.vel_rmse_NL = vel_rmse_NL;
    Results.pos_fit_NL = pos_fit_NL;
    Results.vel_fit_NL = vel_fit_NL;
    Results.pos_rmse_Lin = pos_rmse_Lin;
    Results.vel_rmse_Lin = vel_rmse_Lin;
    Results.pos_fit_Lin = pos_fit_Lin;
    Results.vel_fit_Lin = vel_fit_Lin;
    
    if exist('./Results/Model_Comparison_Results.mat', 'file')
        delete('./Results/Model_Comparison_Results.mat');
    end
    
    save('./Results/Model_Comparison_Results.mat', 'Results', 'Ts');
    
    fprintf('\nResults saved successfully!\n');
end

%%
Time_Duration = toc;
fprintf ("\nCalculations took %.2f seconds\n\n", Time_Duration);

%% COST FUNCTION FOR OPTIMIZATION

function cost = computeCost(params, Time, VelCmd, Pos_exp, PosDot_exp)
    % Extract parameters
    amax = params(1);
    Tr = params(2);
    Kv = params(3);
    
    % Check for invalid parameters
    if amax <= 0 || Tr <= 0 || Kv <= 0
        cost = 1e10;
        return;
    end
    
    % Simulate non-linear model
    Params_NL = [amax, Tr, Kv];
    State0_NL = [Pos_exp(1); PosDot_exp(1)];
    
    try
        % Use tighter tolerances
        opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.01);
        [~, States_NL] = ode45(@(SolverTime, State)CartRateLimiterModel(SolverTime, State, Params_NL, VelCmd, Time), ...
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
        w_rmse_pos = 0.3;
        w_rmse_vel = 1.0;    % Velocity is most important
        w_max_pos = 0.1;
        w_max_vel = 0.3;
        w_fit_pos = 0.2;
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