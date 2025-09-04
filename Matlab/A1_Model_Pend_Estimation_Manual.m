%% PENDULUM MODEL ESTIMATION - OPTIMIZATION
% FREE OSCILLATIONS - NO INPUT
% Renan Duarte - 16/08/2024
% Modificado para comparar dados experimentais, modelo manual e modelo otimizado

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
l_estimated = g * Tosc0^2 / (4 * pi^2);

%% PLOT 1: EXPERIMENTAL DATA
figure(1);
clf(1);
plot(Time, Theta, 'DisplayName', 'Dados Experimentais');
grid on;
xlim([0, Time(end)]);
title('Resposta Experimental do Pêndulo', 'FontSize', 14);
xlabel('Tempo (s)', 'FontSize', 12);
ylabel('Ângulo Theta (rad)', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 11);

%% MODELO MANUAL (DEFINIDO PELO USUÁRIO)

m_manual = 0.146;
l_manual = 0.479;
kd_manual = 200e-006;   % Damping linear moderado
kdr_manual = 5e-006; % Damping quadrático baixo
kc_manual = 100e-006;  % Coulomb friction baixo
    
ParamsManual = [m_manual, l_manual, kd_manual, kdr_manual, kc_manual];

% Resolver modelo manual
[~, StatesManual] = ode45(@(SolverTime, State)CartPendModel(SolverTime, State, ParamsManual, Accel, Time), Time, State0);
ThetaManual = StatesManual(:, 3);

%% PLOT 2: COMPARAÇÃO EXPERIMENTAL vs MODELO MANUAL
figure(2);
clf(2);
plot(Time, Theta, 'DisplayName', 'Dados Experimentais');
hold on;
plot(Time, ThetaManual, 'DisplayName', 'Modelo Manual');
grid on;
xlim([0, Time(end)]);
title('Comparação: Experimental vs Modelo Manual', 'FontSize', 14);
xlabel('Tempo (s)', 'FontSize', 12);
ylabel('Ângulo Theta (rad)', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 11);

%% MODELO OTIMIZADO

m_opt = 0.146;
l_opt = 389.766899652019e-003;
kd_opt = 266.885830984858e-006;
kdr_opt = 4.28381090081854e-006;
kc_opt = 114.048877889130e-006;

ParamsOpt = [m_opt, l_opt, kd_opt, kdr_opt, kc_opt];

% Resolver modelo otimizado
[~, StatesOpt] = ode45(@(SolverTime, State)CartPendModel(SolverTime, State, ParamsOpt, Accel, Time), Time, State0);
ThetaOpt = StatesOpt(:, 3);

%% PLOT 3: COMPARAÇÃO COMPLETA (EXPERIMENTAL vs MANUAL vs OTIMIZADO)
figure(3);
clf(3);
plot(Time, Theta, 'DisplayName', 'Dados Experimentais');
hold on;
plot(Time, ThetaOpt, 'DisplayName', 'Modelo Otimizado');
grid on;
xlim([0, Time(end)]);
title('Comparação Completa: Experimental vs Modelos', 'FontSize', 14);
xlabel('Tempo (s)', 'FontSize', 12);
ylabel('Ângulo Theta (rad)', 'FontSize', 12);
legend('Location', 'best');
set(gca, 'FontSize', 11);

%% END

% Stop the timer and display the total execution time
Time_Duration = toc;
fprintf("\nCálculos completados em %.2f segundos \n\n", Time_Duration);