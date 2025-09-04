%% PENDULUM MODEL ESTIMATION - OPTIMIZATION
% FREE OSCILLATIONS - NO INPUT
% Renan Duarte - 16/08/2024

% Weigths of frequency and decay error can be adjusted in the minimization
% funcion at the end of the file

%% INITIAL SETUP

% Set format to long engineering notation for more precise output
format long eng;
clear all;  % Clear all variables from the workspace
close all;  % Close all figure windows
clc;        % Clear command window

% Add folders containing data, functions, and results to the MATLAB path
addpath ('./Datasources/');  % Folder for data sources
addpath ('./Functions/');    % Folder for custom functions
addpath ('./Results/');      % Folder for saving results

% Dialog popup to ask user if they want to save the results
OptSave = SavePopUp;

% Constant for gravitational acceleration [m/s^2]
g = 9.81;

% Start a timer to measure the execution time of the script
tic;

%% EXPERIMENTAL DATA

% Import experimental data from an Excel file
Experimental_Data = readtable('A_Model_Pend_Estimation.xlsx', 'Sheet', 1);

% Extract parameters from the data table
Time = Experimental_Data.Time;
Accel = Experimental_Data.Accel;
Pos = Experimental_Data.Pos;
PosDot = Experimental_Data.PosDot;
Theta = Experimental_Data.Theta;
ThetaDot = Experimental_Data.ThetaDot;

% Adjust theta values to oscillate around zero
for i = 1:numel(Theta)
    if (Theta(i) < 0)
        Theta(i) = Theta(i) + pi;  % Shift negative values up by π
    elseif (Theta(i) > 0)
        Theta(i) = Theta(i) - pi;  % Shift positive values down by π
    end
end

% Further adjustment to make theta oscillate around -π
Theta = Theta - pi;

% If no input is given, make it a single value to speedup calculations
if sum(Accel) == 0
    Accel = 0;
end

% Initial state vector
State0 = [Pos(1); PosDot(1); Theta(1); ThetaDot(1)];

%% FINDING OSCILLATION PEAKS

% Find the peaks in the theta data
[Peaks, Idx] = findpeaks(Theta);
TimePeaks = Time(Idx);  % Get the corresponding time points of the peaks

% Filter peaks based on a threshold around -π to exclude outliers
IdxValid = (abs(Peaks + pi) < 0.15) & (abs(Peaks + pi) > 0.1);

% Keep only the valid peaks and their corresponding times
Peaks = Peaks(IdxValid);
TimePeaks = TimePeaks(IdxValid);

% Estimate the undumped period of oscillation (Tosc0) from the time between peaks
Tosc0 = mean(diff(TimePeaks));

%% EQUIVALENT IDEAL PENDULUM ESTIMATION

% Estimate the effective length of the pendulum using the period Tosc0
l = g * Tosc0^2 / (4 * pi^2);

%% PENDULUM PARAMETERS - RANGES OR SINGLE VALUES

% Define ranges or single values for the parameters to be optimized:
% m -> Pendulum mass [kg]
% l -> Distance from the hinge point to the center of gravity [m]
% kd  -> Linear damping coefficient [Ns/m] 
% kdr -> Air drag constant [Ns^2/m^2]
% kc  -> Coulomb constant force [N] 

m = 0.146;
l = 0.479;
kd = 266.885830984858e-006;
kdr = 4.28381090081854e-006;
kc = [1e-12 1e-1];

% Set the lower and upper bounds for the optimization
LowerBound = [min(m), min(l), min(kd), min(kdr), min(kc)];
UpperBound = [max(m), max(l), max(kd), max(kdr), max(kc)];

% Initial parameter values for optimization (starting point)
Param0 = LowerBound; 

% Define the objective function for minimization, which simulates the pendulum
objectiveFunction = @(Params) MinimizationFunction(Params, Accel, Time, State0, Theta);

%% OPTIMIZATION

% Set options for the optimization solver (lsqnonlin)
options = optimoptions('lsqnonlin', 'Display', 'iter', ...
    'StepTolerance', 1e-20, ...
    'OptimalityTolerance', 1e-15, ...
    'FunctionTolerance', 1e-25, ...
    'MaxFunctionEvaluations', 1e3, ...
    'MaxIterations', 1e3);

% Run the optimization to find the best-fitting parameters
ParamsOpt = lsqnonlin(objectiveFunction, Param0, LowerBound, UpperBound, options);

%% SOLVE MODEL WITH BEST PARAMETERS

% Extract optimized parameters for the pendulum
m = ParamsOpt(1)
l = ParamsOpt(2)

% Extract optimized damping coefficients
kd = ParamsOpt(3)
kdr = ParamsOpt(4)
kc = ParamsOpt(5)

% Solve the nonlinear pendulum model using the optimized parameters
[~, States] = ode45(@(SolverTime, State)CartPendModel(SolverTime, State, ParamsOpt, Accel, Time), Time, State0);

% Extract the angular displacement (theta) and velocity from the solution
ThetaM = States(:, 3);
ThetaDotM = States(:, 4);

% Calculate the frequencies of the oscillations for the experimental data and model
[p f] = pspectrum(Theta - mean(Theta), Time);
[pM fM] = pspectrum(ThetaM - mean(ThetaM), Time);

% Calculate a decay envelope for the experimental data and model
Env = envelope(Theta, numel(Theta), 'analytic');
EnvM = envelope(ThetaM, numel(ThetaM), 'analytic');

% Show errors
ErrorFreq = 1 - R2_coeff(p, pM)
ErrorDecay = 1 - R2_coeff(Env(100:end-100), EnvM(100:end-100)) 
        
%% PLOTS

% Plot the experimental data and the simulated model results for comparison
hFig = figure(1);
clf(1);

subplot(2, 2, [1 2]);
plot(Time, Theta, 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaM, 'DisplayName', 'NL model');
grid on;
xlim([0, Time(end)]);
title('Time response');
xlabel('Time (s)');
ylabel('Theta (rad)');
legend;

subplot(2, 2, 3);
plot(f, p, 'DisplayName', 'Experimental');
hold on;
plot(fM, pM, 'DisplayName', 'NL Model');
grid on;
xlim([1/Tosc0 - 0.5, 1/Tosc0 + 0.5]);
title('Oscillation frequencies');
xlabel('Frequency (Hz)');
ylabel('Power');
legend;

subplot(2, 2, 4);
plot(Time, Env, 'DisplayName', 'Experimental');
hold on;
plot(Time, EnvM, 'DisplayName', 'NL Model');
grid on;
xlim([0, Time(end)]);
title('Decay');
xlabel('Time (s)');
ylabel('Theta (rad)');
legend;

%% SAVE PENDULUM OBJECT

% If the user chose to save the results, save the optimized parameters
if OptSave    
    Pendulum.m = m;
    Pendulum.l = l;
    Pendulum.kd = kd;
    Pendulum.kdr = kdr;
    Pendulum.kc = kc;
    
    % Delete the old results file if it exists and save the new one
    if exist('./Results/Pendulum.mat', 'file')
        delete('./Results/Pendulum.mat');
    end
    save('./Results/Pendulum.mat', 'Pendulum');    
end

%% END

% Stop the timer and display the total execution time
Time_Duration = toc;
fprintf("Calculations took %.2f seconds \n\n", Time_Duration);

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
    % Solve the pendulum's ODE with the given parameters
    [~, States] = ode45(@(SolverTime, State)CartPendModel(SolverTime, State, Params, Input, Time), Time, State0);
    
    % Extract the model's predicted angular displacement
    ThetaM = States(:, 3);

%    Error = 1 - R2_coeff (ThetaExp, ThetaM);
    
    % Calculate the frequencies of the oscillations for the experimental data and model
    [p, ~] = pspectrum(ThetaExp - mean(ThetaExp), Time);
    [pM, ~] = pspectrum(ThetaM - mean(ThetaM), Time);

    % Calculate the decay envelope for the experimental data and model
    Env = envelope(ThetaExp, numel(ThetaExp), 'analytic');
    EnvM = envelope(ThetaM, numel(ThetaM), 'analytic');
    
    % Calculate the total error as the sum of errors in peak times and envelopes
    Error = 0.1*(1 - R2_coeff(p, pM)) + ...
            0.9*(1 - R2_coeff(Env(100:end-100), EnvM(100:end-100)));
end
