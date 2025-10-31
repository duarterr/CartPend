%% PENDULUM MODEL ESTIMATION - OPTIMIZATION
% FREE OSCILLATIONS - NO INPUT
% Renan Duarte 26/10/2025

% Minimization function compares oscillation frequency and decay envelope of
% experimental data with the non-linear model. For small-signal oscillations
% around the downward equilibrium position (theta = pi), the equivalent 
% pendulum length can be analytically determined by l = g*Tosc0^2/(4*pi^2),
% where Tosc0 is the natural period of oscillation. 

format long eng;
clear all;
close all;
clc;

% Search stuff also in this folder
addpath ('./Datasources/');
addpath ('./Functions/');
addpath ('./Results/');

% Linearize and compare with linear model
LinearizeModel = true;  % Set to false to skip linear model analysis

% Constant for gravitational acceleration [m/s^2]
g = 9.81;

% Start counting time
tic;

%% EXPERIMENTAL DATA

% Import data
try    
    Exp = readtable('CartPend_Free_Oscilations3.xlsx', 'Sheet', 1);
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
Theta = Exp.Pendulum_Pos_Filt;
ThetaDot = Exp.Pendulum_Vel_Filt;

% Adjust pendulum angle
ThetaEq = mean(Theta(end-100:end));
Theta = wrapToPi(Theta - ThetaEq) + pi;  

% Filter angle a little
% Theta = smooth(Theta, 6);

fprintf('DATASET INFO (before trimming)\n\n');

fprintf('Data duration: %.2f s\n', Time(end) - Time(1));
fprintf('Number of samples: %d\n\n', length(Time));

%% FIND START POINT BASED ON AMPLITUDE

% Define the starting amplitude threshold (rad from equilibrium)
Amp_start = deg2rad(90);  % Start when |theta - pi| <= Amp_start (amplitude decaying TO this value)

fprintf('Searching for starting point where amplitude <= %.3f rad (%.2f deg from equilibrium)...\n', ...
    Amp_start, Amp_start*180/pi);

% Find LOCAL MAXIMA where theta_dot crosses zero from positive to negative
% This ensures we're at a peak with theta_dot ≈ 0

idx_start = [];

for i = 2:length(ThetaDot)-1
    % Check if this is a zero-crossing of velocity (peak of position)
    % ThetaDot changes from positive to negative
    if ThetaDot(i-1) > 0 && ThetaDot(i+1) < 0
        % This is approximately a maximum (peak)
        % Check if amplitude is BELOW the threshold (decayed enough)
        amplitude = abs(Theta(i) - pi);
        
        if amplitude <= Amp_start
            idx_start = i;
            fprintf('Found starting point at index %d:\n', idx_start);
            fprintf('  Time: %.3f s\n', Time(idx_start) - Time(1));
            fprintf('  Theta: %.4f rad\n', Theta(idx_start));
            fprintf('  Amplitude: %.4f rad (%.2f deg from equilibrium)\n', ...
                amplitude, amplitude*180/pi);
            fprintf('  Theta_dot: %.6f rad/s (close to zero)\n', ThetaDot(idx_start));
            break;
        end
    end
end

% If still no point found, use beginning of data
if isempty(idx_start)
    warning('No starting point found with amplitude <= %.3f rad and velocity ≈ 0.', Amp_start);
    fprintf('Minimum amplitude in dataset: %.4f rad (%.2f deg from equilibrium)\n\n', ...
        min(abs(Theta - pi)), min(abs(Theta - pi))*180/pi);
    idx_start = 1;
else
    fprintf('\nTrimming %d points (%.2f seconds) from beginning of dataset.\n\n', ...
        idx_start-1, Time(idx_start));
end

% Trim data to start from the identified point
Time = Time(idx_start:end);
VelCmd = VelCmd(idx_start:end);
PWM = PWM(idx_start:end);
Pos = Pos(idx_start:end);
PosDot = PosDot(idx_start:end);
Theta = Theta(idx_start:end);
ThetaDot = ThetaDot(idx_start:end);

% Reset time vector to start at zero
Time = Time - Time(1);

% Get number of samples
N = length(Pos);

% Calculate average sample time
Ts = mean(diff(Time));

% Recreate uniform time vector
Time = (0:N-1)' * Ts;

fprintf('DATASET INFO (after trimming)\n\n');

fprintf('Sample time: %.6f s (%.1f Hz)\n', Ts, 1/Ts);
fprintf('Data duration: %.2f s\n', Time(end));
fprintf('Number of samples: %d\n', length(Time));
fprintf('Initial conditions: theta = %.4f rad, theta_dot = %.6f rad/s\n\n', ...
    Theta(1), ThetaDot(1));

%%

% Calculate derivatives for validation
PosDotCalc = gradient(Pos(:)) ./ gradient(Time(:)); 
AccelCalc = gradient(PosDotCalc(:)) ./ gradient(Time(:)); 
ThetaDotCalc = gradient(Theta(:)) ./ gradient(Time(:));

% Replace original velocity data
ThetaDot = ThetaDotCalc;

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

%% FINDING OSCILLATION FREQUENCY

% Estimate the undamped period of oscillation (Tosc0)
[p, f] = pspectrum(Theta - mean(Theta), Time);
Tosc0 = 1/f(p == max(p));

fprintf('Estimated oscillation period: %.4f s (%.2f Hz)\n', Tosc0, 1/Tosc0);

% Estimate the effective length of the pendulum using the period Tosc0
l_estimated = g * Tosc0^2 / (4 * pi^2);

fprintf('Estimated pendulum length: %.4f m\n\n', l_estimated);

%% AUTOMATIC PARAMETER TUNING

fprintf('AUTOMATIC PARAMETER TUNING \n');

% Parallel pool
if isempty(gcp('nocreate'))
    parpool('local');
end

% Parameters to be optimized - Ranges [min max] or single value
m = 0.146;
l = l_estimated;
kd = [1e-4 1e-2];
kdr = [1e-8 1e-3];
kc = [1e-8 1e-3];

% Set the lower and upper bounds for the optimization
LowerBound = [min(m), min(l), min(kd), min(kdr), min(kc)];
UpperBound = [max(m), max(l), max(kd), max(kdr), max(kc)];

% Initial parameter values for optimization (starting point)
Param0 = LowerBound;

% Initial state vector
State0 = [Theta(1); ThetaDot(1)];

% Resample angle and time to use less points
[ThetaResample,TimeResample] = resample(Theta, Time, f(p == max(p))*10);
Accel = zeros(size(TimeResample));

% Define the objective function for minimization, which simulates the pendulum
objectiveFunction = @(Params) MinimizationFunction(Params, Accel, TimeResample, State0, ThetaResample);

% Method: lsqnonlin with Levenberg-Marquardt algorithm
options = optimoptions('lsqnonlin', ...
    'Display', 'iter', ...
    'Algorithm', 'Levenberg-Marquardt', ...
    'StepTolerance', 1e-20, ...
    'OptimalityTolerance', 1e-15, ...
    'FunctionTolerance', 1e-15, ...
    'MaxFunctionEvaluations', 1e3, ...
    'MaxIterations', 200, ...
    'UseParallel', true, ...
    'SpecifyObjectiveGradient', false);

% Run the optimization to find the best-fitting parameters
ParamsOpt = lsqnonlin(objectiveFunction, Param0, LowerBound, UpperBound, options);

fprintf('Params: [m=%.6f, l=%.6f, kd=%.6e, kdr=%.6e, kc=%.6e]\n\n', ...
    ParamsOpt(1), ParamsOpt(2), ParamsOpt(3), ParamsOpt(4), ParamsOpt(5));

% Extract optimized parameters
m = ParamsOpt(1);
l = ParamsOpt(2);
kd = ParamsOpt(3);
kdr = ParamsOpt(4);
kc = ParamsOpt(5);

%% FINAL PARAMETERS

fprintf('FINAL MODEL PARAMETERS \n\n');
fprintf('Pendulum:\n');
fprintf('  m   = %.6f kg\n', m);
fprintf('  l   = %.6f m\n', l);
fprintf('  kd  = %.6e Ns/m\n', kd);
fprintf('  kdr = %.6e Ns^2/m^2\n', kdr);
fprintf('  kc  = %.6e N\n\n', kc);

%% SIMULATE NON-LINEAR MODEL WITH OPTIMIZED PARAMETERS

% Solve the nonlinear pendulum model using the optimized parameters
Accel = zeros(size(Time));
[~, States_NL] = ode45(@(SolverTime, State)PendNonLinearModel(SolverTime, State, ParamsOpt, Accel, Time), Time, State0);

% Extract the angular displacement (theta) and velocity from the solution
ThetaM_NL = States_NL(:, 1);
ThetaDotM_NL = States_NL(:, 2);

%% CALCULATE ERRORS - NON-LINEAR MODEL

% Angle errors
theta_error_NL = Theta - ThetaM_NL;
thetadot_error_NL = ThetaDot - ThetaDotM_NL;

% Percentage errors
theta_error_percent_NL = 100 * theta_error_NL ./ Theta;
thetadot_error_percent_NL = 100 * thetadot_error_NL ./ ThetaDot;

% Replace inf/nan values (division by zero) with zero
theta_error_percent_NL(~isfinite(theta_error_percent_NL)) = 0;
thetadot_error_percent_NL(~isfinite(thetadot_error_percent_NL)) = 0;

theta_rmse_NL = sqrt(mean(theta_error_NL.^2));
thetadot_rmse_NL = sqrt(mean(thetadot_error_NL.^2));

theta_fit_NL = 100 * (1 - theta_rmse_NL/std(Theta));
thetadot_fit_NL = 100 * (1 - thetadot_rmse_NL/std(ThetaDot));

%% FREQUENCY AND DECAY ANALYSIS - NON-LINEAR

% Calculate the frequencies of the oscillations for the experimental data and model
[p, f] = pspectrum(Theta - mean(Theta), Time);
[pM_NL, fM_NL] = pspectrum(ThetaM_NL - mean(ThetaM_NL), Time);

% Calculate a decay envelope for the experimental data and model
Env = envelope(Theta, numel(Theta), 'analytic');
EnvM_NL = envelope(ThetaM_NL, numel(ThetaM_NL), 'analytic');

% Calculate R2 coefficients
R2_freq_NL = R2_coeff(p, pM_NL);
R2_decay_NL = R2_coeff(Env(100:end-100), EnvM_NL(100:end-100));

ErrorFreq_NL = 1 - R2_freq_NL;
ErrorDecay_NL = 1 - R2_decay_NL;

%% LINEARIZED MODEL - DERIVATION AND OPTIMIZATION

if LinearizeModel == true
    fprintf('LINEARIZED MODEL DERIVATION AND OPTIMIZATION\n\n');  
    
    % STEP 1: LINEARIZE THE NON-LINEAR MODEL AROUND THETA = PI
    % From the non-linear model:
    % theta_ddot = (m*g*l/I)*sin(theta) - (m*l/I)*cos(theta)*accel 
    %              - kd*theta_dot/I - kdr*theta_dot^2/I - kc*sign(theta_dot)/I
    
    % Linearization for small phi = theta - pi:
    % - sin(pi + phi) ≈ -phi
    % - cos(pi + phi) ≈ -1
    % - theta_dot^2 ≈ 0 (neglect kdr for small oscillations)
    % - kc*sign(theta_dot) ≈ kc_eq*theta_dot (Coulomb friction becomes equivalent viscous damping)
    
    % For harmonic oscillations with amplitude A and frequency omega:
    % Equivalent viscous damping from Coulomb friction: kd_eq ≈ 4*kc/(pi*omega*A)
    % In the linear model, we'll let kd_lin absorb both viscous and Coulomb effects
    
    fprintf('Deriving linearized model from non-linear dynamics...\n');
    fprintf('Linearization point: theta = pi (pendulum down)\n');
    fprintf('Approximations:\n');
    fprintf('  - Small angle: phi = theta - pi\n');
    fprintf('  - Neglect air drag: kdr*theta_dot^2 ≈ 0\n');
    fprintf('  - Coulomb friction absorbed into equivalent viscous damping\n\n');
    
    % Estimate typical amplitude and frequency for Coulomb friction linearization
    A_typical = std(Theta - pi);
    omega_typical = 2*pi/Tosc0;
    kd_coulomb_equiv = 4*kc / (pi * omega_typical * A_typical);
    kd_total = kd + kd_coulomb_equiv;
    
    fprintf('Coulomb friction linearization:\n');
    fprintf('  Typical amplitude A = %.4f rad\n', A_typical);
    fprintf('  Typical frequency ω = %.4f rad/s\n', omega_typical);
    fprintf('  Equivalent viscous damping from Coulomb: kd_eq = %.6e Ns/m\n', kd_coulomb_equiv);
    fprintf('  Total expected damping: kd_total ≈ %.6e Ns/m\n\n', kd_total);
    
    % Initial guess for linear parameters [m_lin, l_lin, kd_lin]
    % Start with non-linear optimized values but allow them to vary
    m_lin = m;
    l_lin = l;%[l-0.05 l+0.05];
    kd_lin = [kd kd_total];
    
    % Bounds for linear parameters
    lb_lin = [min(m_lin), min(l_lin), min(kd_lin)];
    ub_lin = [max(m_lin), max(l_lin), max(kd_lin)];
    params_lin_initial = lb_lin;
    
    State0_Lin = [Theta(1) - pi; ThetaDot(1)];

    % Define objective function for linear model
    objectiveFunction_Lin = @(Params) MinimizationFunctionLinear(Params, Time, State0_Lin, Theta);
    
    % Optimization options - usando trust-region-reflective (melhor para bounded problems)
    options_lin = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'Algorithm', 'sqp', ...
        'StepTolerance', 1e-12, ...
        'OptimalityTolerance', 1e-10, ...
        'ConstraintTolerance', 1e-10, ...
        'MaxFunctionEvaluations', 500, ...
        'MaxIterations', 100, ...
        'FiniteDifferenceStepSize', 1e-8);

    % Run optimization for linear model
    ParamsOpt_Lin = fmincon(objectiveFunction_Lin, params_lin_initial, ...
        [], [], [], [], lb_lin, ub_lin, [], options_lin);

    fprintf('Optimized linear params: [m=%.6f, l=%.6f, kd=%.6e]\n\n', ...
        ParamsOpt_Lin(1), ParamsOpt_Lin(2), ParamsOpt_Lin(3));
    
    % Extract optimized linear parameters
    m_lin = ParamsOpt_Lin(1);
    l_lin = ParamsOpt_Lin(2);
    kd_lin = ParamsOpt_Lin(3);
    
    % STEP 3: BUILD LINEAR STATE-SPACE MODEL WITH OPTIMIZED PARAMETERS
    I_lin = m_lin * l_lin^2;
    a_lin = -m_lin*g*l_lin/I_lin;
    b_lin = -kd_lin/I_lin;
    c_lin = m_lin*l_lin/I_lin;
    
    A_lin = [0,     1; 
             a_lin, b_lin];
         
    B_lin = [0; 
             c_lin];
         
    C_lin = [1, 0];
    D_lin = 0;
    
    sys_lin = ss(A_lin, B_lin, C_lin, D_lin);
    
    fprintf('Optimized linear model parameters:\n');
    fprintf('  m_lin   = %.6f kg\n', m_lin);
    fprintf('  l_lin   = %.6f m\n', l_lin);
    fprintf('  kd_lin  = %.6e Ns/m\n', kd_lin);
    fprintf('  I_lin   = %.6e kg*m^2\n\n', I_lin);
    
    fprintf('Linear model state-space matrices:\n');
    fprintf('A = [%f, %f;\n     %f, %f]\n\n', A_lin(1,1), A_lin(1,2), A_lin(2,1), A_lin(2,2));
    fprintf('B = [%f;\n     %f]\n\n', B_lin(1), B_lin(2));
    
    % STEP 4: SIMULATE OPTIMIZED LINEAR MODEL
    % Initial condition in phi coordinates
    phi0 = Theta(1) - pi;
    phi_dot0 = ThetaDot(1);
    x0_lin = [phi0; phi_dot0];
    
    % Simulate
    Accel_input = zeros(size(Time));
    [~, ~, States_Lin] = lsim(sys_lin, Accel_input, Time, x0_lin);
    
    % Convert back to theta coordinates
    Phi_Lin = States_Lin(:, 1);
    PhiDot_Lin = States_Lin(:, 2);
    ThetaM_Lin = Phi_Lin + pi;
    ThetaDotM_Lin = PhiDot_Lin;
    
    % Check for stability
    if any(~isfinite(ThetaM_Lin)) || any(~isfinite(ThetaDotM_Lin))
        fprintf('Warning: Linear model produced non-finite values\n\n');
    end

%% CALCULATE ERRORS - LINEAR MODEL

    % Angle errors
    theta_error_Lin = Theta - ThetaM_Lin;
    thetadot_error_Lin = ThetaDot - ThetaDotM_Lin;
    
    % Percentage errors
    theta_error_percent_Lin = 100 * theta_error_Lin ./ Theta;
    thetadot_error_percent_Lin = 100 * thetadot_error_Lin ./ ThetaDot;
    
    % Replace inf/nan values (division by zero) with zero
    theta_error_percent_Lin(~isfinite(theta_error_percent_Lin)) = 0;
    thetadot_error_percent_Lin(~isfinite(thetadot_error_percent_Lin)) = 0;
    
    theta_rmse_Lin = sqrt(mean(theta_error_Lin.^2));
    thetadot_rmse_Lin = sqrt(mean(thetadot_error_Lin.^2));
    
    theta_fit_Lin = 100 * (1 - theta_rmse_Lin/std(Theta));
    thetadot_fit_Lin = 100 * (1 - thetadot_rmse_Lin/std(ThetaDot));

%% FREQUENCY AND DECAY ANALYSIS - LINEAR

    % Calculate the frequencies of the oscillations for the linear model
    [pM_Lin, fM_Lin] = pspectrum(ThetaM_Lin - mean(ThetaM_Lin), Time);

    % Calculate a decay envelope for the linear model
    EnvM_Lin = envelope(ThetaM_Lin, numel(ThetaM_Lin), 'analytic');

    % Calculate R2 coefficients
    R2_freq_Lin = R2_coeff(p, pM_Lin);
    R2_decay_Lin = R2_coeff(Env(100:end-100), EnvM_Lin(100:end-100));

    ErrorFreq_Lin = 1 - R2_freq_Lin;
    ErrorDecay_Lin = 1 - R2_decay_Lin;

end

%% PRINT RESULTS

fprintf('VALIDATION RESULTS\n\n');

fprintf('NON-LINEAR MODEL:\n');
fprintf('  ANGLE:\n');
fprintf('    RMSE: %.6f rad (%.2f deg)\n', theta_rmse_NL, theta_rmse_NL*180/pi);
fprintf('    FIT:  %.2f %%\n', theta_fit_NL);
fprintf('  ANGULAR VELOCITY:\n');
fprintf('    RMSE: %.6f rad/s\n', thetadot_rmse_NL);
fprintf('    FIT:  %.2f %%\n', thetadot_fit_NL);
fprintf('  FREQUENCY MATCH:\n');
fprintf('    R²:    %.4f\n', R2_freq_NL);
fprintf('    Error: %.4f\n', ErrorFreq_NL);
fprintf('  DECAY ENVELOPE:\n');
fprintf('    R²:    %.4f\n', R2_decay_NL);
fprintf('    Error: %.4f\n\n', ErrorDecay_NL);

if LinearizeModel == true
    fprintf('LINEAR MODEL:\n');
    fprintf('  ANGLE:\n');
    fprintf('    RMSE: %.6f rad (%.2f deg)\n', theta_rmse_Lin, theta_rmse_Lin*180/pi);
    fprintf('    FIT:  %.2f %%\n', theta_fit_Lin);
    fprintf('  ANGULAR VELOCITY:\n');
    fprintf('    RMSE: %.6f rad/s\n', thetadot_rmse_Lin);
    fprintf('    FIT:  %.2f %%\n', thetadot_fit_Lin);
    fprintf('  FREQUENCY MATCH:\n');
    fprintf('    R²:    %.4f\n', R2_freq_Lin);
    fprintf('    Error: %.4f\n', ErrorFreq_Lin);
    fprintf('  DECAY ENVELOPE:\n');
    fprintf('    R²:    %.4f\n', R2_decay_Lin);
    fprintf('    Error: %.4f\n\n', ErrorDecay_Lin);
    
    fprintf('COMPARISON:\n');
    fprintf('  Angle improvement (NL vs Lin): %.2f %%\n', ...
        100*(theta_rmse_Lin - theta_rmse_NL)/theta_rmse_Lin);
    fprintf('  Angular velocity improvement (NL vs Lin): %.2f %%\n', ...
        100*(thetadot_rmse_Lin - thetadot_rmse_NL)/thetadot_rmse_Lin);
    fprintf('  Frequency match improvement (NL vs Lin): %.2f %%\n', ...
        100*(ErrorFreq_Lin - ErrorFreq_NL)/ErrorFreq_Lin);
    fprintf('  Decay envelope improvement (NL vs Lin): %.2f %%\n\n', ...
        100*(ErrorDecay_Lin - ErrorDecay_NL)/ErrorDecay_Lin);
end

%% PLOTS - MODEL COMPARISON

hFig2 = figure(2);
set(hFig2, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);

% Angle comparison
subplot(2, 3, 1);
plot(Time, Theta, 'k', 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaM_NL, 'b--', 'DisplayName', 'Non-Linear');
if LinearizeModel == true
    plot(Time, ThetaM_Lin, 'r:', 'DisplayName', 'Linear');
end
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angle [rad]');
if LinearizeModel == true
    title(sprintf('Pendulum Angle (FIT: NL=%.2f%%, Lin=%.2f%%)', theta_fit_NL, theta_fit_Lin));
else
    title(sprintf('Pendulum Angle (FIT: NL=%.2f%%)', theta_fit_NL));
end
legend('Location', 'best');

% Angular velocity comparison
subplot(2, 3, 2);
plot(Time, ThetaDot, 'k', 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaDotM_NL, 'b--', 'DisplayName', 'Non-Linear');
if LinearizeModel == true
    plot(Time, ThetaDotM_Lin, 'r:', 'DisplayName', 'Linear');
end
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
if LinearizeModel == true
    title(sprintf('Angular Velocity (FIT: NL=%.2f%%, Lin=%.2f%%)', thetadot_fit_NL, thetadot_fit_Lin));
else
    title(sprintf('Angular Velocity (FIT: NL=%.2f%%)', thetadot_fit_NL));
end
legend('Location', 'best');

% Frequency spectrum
subplot(2, 3, 3);
plot(f, p, 'k', 'DisplayName', 'Experimental');
hold on;
plot(fM_NL, pM_NL, 'b--', 'DisplayName', 'Non-Linear');
if LinearizeModel == true
    plot(fM_Lin, pM_Lin, 'r:', 'DisplayName', 'Linear');
end
grid on;
xlim([1/Tosc0 - 0.5, 1/Tosc0 + 0.5]);
xlabel('Frequency [Hz]');
ylabel('Power');
if LinearizeModel == true
    title(sprintf('Frequency Spectrum (R²: NL=%.4f, Lin=%.4f)', R2_freq_NL, R2_freq_Lin));
else
    title(sprintf('Frequency Spectrum (R²: NL=%.4f)', R2_freq_NL));
end
legend('Location', 'best');

% Angle error
subplot(2, 3, 4);
plot(Time, theta_error_percent_NL, 'b-', 'DisplayName', 'Non-Linear');
hold on;
if LinearizeModel == true
    plot(Time, theta_error_percent_Lin, 'r-', 'DisplayName', 'Linear');
end
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [%]');
if LinearizeModel == true
    title(sprintf('Angle Error (RMSE: NL=%.6f, Lin=%.6f)', theta_rmse_NL, theta_rmse_Lin));
else
    title(sprintf('Angle Error (RMSE: NL=%.6f)', theta_rmse_NL));
end
legend('Location', 'best');

% Angular velocity error
subplot(2, 3, 5);
plot(Time, thetadot_error_percent_NL, 'b-', 'DisplayName', 'Non-Linear');
hold on;
if LinearizeModel == true
    plot(Time, thetadot_error_percent_Lin, 'r-', 'DisplayName', 'Linear');
end
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Error [%]');
if LinearizeModel == true
    title(sprintf('Angular Velocity Error (RMSE: NL=%.6f, Lin=%.6f)', thetadot_rmse_NL, thetadot_rmse_Lin));
else
    title(sprintf('Angular Velocity Error (RMSE: NL=%.6f)', thetadot_rmse_NL));
end
legend('Location', 'best');

% Decay envelope
subplot(2, 3, 6);
plot(Time, Env, 'k', 'DisplayName', 'Experimental');
hold on;
plot(Time, EnvM_NL, 'b--', 'DisplayName', 'Non-Linear');
if LinearizeModel == true
    plot(Time, EnvM_Lin, 'r:', 'DisplayName', 'Linear');
end
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Envelope [rad]');
if LinearizeModel == true
    title(sprintf('Decay Envelope (R²: NL=%.4f, Lin=%.4f)', R2_decay_NL, R2_decay_Lin));
else
    title(sprintf('Decay Envelope (R²: NL=%.4f)', R2_decay_NL));
end
legend('Location', 'best');

sgtitle('Pendulum Model Validation: Non-Linear vs Linear vs Experimental');

%% SAVE PENDULUM OBJECT

% Dialog popup - Save results
OptSave = SavePopUp;

% If the user chose to save the results, save the optimized parameters
if OptSave    
    Pendulum.m = m;
    Pendulum.l = l;
    Pendulum.kd = kd;
    Pendulum.kdr = kdr;
    Pendulum.kc = kc;
    
    PendulumLinear.m = m_lin;
    PendulumLinear.l = l_lin;
    PendulumLinear.kd = kd_lin;
    
    % Delete the old results file if it exists and save the new one
    if exist('./Results/Pendulum.mat', 'file')
        delete('./Results/Pendulum.mat');
    end
    save('./Results/Pendulum.mat', 'Pendulum', 'PendulumLinear');    
end

Time_Duration = toc;
fprintf ("\nCalculations took %.2f seconds\n\n", Time_Duration);

%% CORRELATION BETWEEN TWO DATA SETS

function R2 = R2_coeff(data, data_fit)
    % Compute the R^2 correlation coefficient between two data sets
    
    % Total sum of squares (variance of the data)
    sum_of_squares = sum((data - mean(data)).^2);
    
    % Residual sum of squares (variance of the residuals)
    sum_of_squares_of_residuals = sum((data - data_fit).^2);
    
    % R^2 is the proportion of the variance explained by the model
    R2 = 1 - sum_of_squares_of_residuals / sum_of_squares;
end

%% MINIMIZATION FUNCTION

function Error = MinimizationFunction(Params, Input, Time, State0, ThetaExp)
    % Pre-compute experimental spectrum and envelope ONCE outside this function
    % Pass them as parameters instead of ThetaExp
    
    persistent p_exp Env_exp
    
    if isempty(p_exp)
        [p_exp, ~] = pspectrum(ThetaExp - mean(ThetaExp), Time);
        Env_exp = envelope(ThetaExp, numel(ThetaExp), 'analytic');
        Env_exp = Env_exp(100:end-100);  % Pre-crop
    end
    
    w_freq = 0.05;
    w_decay = 0.95;
    
    try
        opts = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
        [~, States] = ode45(@(t,y)PendNonLinearModel(t, y, Params, Input, Time), ...
                            Time, State0, opts);
        
        ThetaM = States(:, 1);
        
        if any(~isfinite(ThetaM)) || max(abs(ThetaM - pi)) > 10
            Error = 1e10;
            return;
        end
        
        [pM, ~] = pspectrum(ThetaM - mean(ThetaM), Time);
        EnvM = envelope(ThetaM, numel(ThetaM), 'analytic');
        EnvM = EnvM(100:end-100);
        
        R2_freq = R2_coeff(p_exp, pM);
        R2_decay = R2_coeff(Env_exp, EnvM);
        
        Error = w_freq * (1 - R2_freq) + w_decay * (1 - R2_decay);
        
    catch
        Error = 1e10;
    end
end

%% MINIMIZATION FUNCTION FOR LINEAR MODEL

function Error = MinimizationFunctionLinear(Params, Time, State0, ThetaExp)
    % Optimization function for linearized pendulum model
    % Params: [m_lin, l_lin, kd_lin]
    % OBJECTIVE: Maximize R2_freq and R2_decay
    
    % Weights for multi-objective optimization
    w_freq = 0.05;   % Weight for frequency match
    w_decay = 0.95;  % Weight for decay envelope (more important)
            
    % Extract parameters
    m_lin = Params(1);
    l_lin = Params(2);
    kd_lin = Params(3);
    
    % Build linear model
    g = 9.81;
    I_lin = m_lin * l_lin^2;
    
    A_lin = [0,                          1; 
             -m_lin*g*l_lin/I_lin,  -kd_lin/I_lin];
         
    B_lin = [0; 
             m_lin*l_lin/I_lin];
    
    C_lin = [1, 0];
    D_lin = 0;
    
    sys_lin = ss(A_lin, B_lin, C_lin, D_lin);
    
    % Simulate
    Accel_input = zeros(size(Time));
    
    try
        [States, ~] = lsim(sys_lin, Accel_input, Time, State0);
        
        % Extract phi (deviation from pi)
        Phi_M = States(:, 1);
        
        % Convert to theta
        ThetaM = Phi_M + pi;
        
        % Check for instability
        if any(~isfinite(ThetaM)) || max(abs(Phi_M)) > 10
            Error = 1e10;
            return;
        end
        
        % Calculate frequency spectrum match
        [p, ~] = pspectrum(ThetaExp - mean(ThetaExp), Time);
        [pM, ~] = pspectrum(ThetaM - mean(ThetaM), Time);
        
        % Calculate decay envelope match
        Env = envelope(ThetaExp, numel(ThetaExp), 'analytic');
        EnvM = envelope(ThetaM, numel(ThetaM), 'analytic');
        
        % Compute R2 coefficients
        R2_freq = R2_coeff(p, pM);
        R2_decay = R2_coeff(Env(100:end-100), EnvM(100:end-100));
        
        % MAXIMIZE R2 = MINIMIZE (1 - R2)
        % This way: R2=1 (perfect) gives Error=0, R2=0 gives Error=1
        Error = w_freq * (1 - R2_freq) + w_decay * (1 - R2_decay);
    catch Me
        warning('Linear simulation failed: %s', ME.message);
        Error = 1e10;
    end
end