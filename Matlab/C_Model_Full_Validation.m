%% PENDULUM MODEL VALIDATION
% Renan Duarte - 30/04/2024

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

% Start a timer to measure the execution time of the script
tic;

%%

STEPPER_STEPS_REV = 200;
STEPPER_MICROSTEPS = 32;
STEPPER_PPR = (STEPPER_STEPS_REV*STEPPER_MICROSTEPS);
STEPPER_PD = 0.0143859964587984;
STEPPER_KV = (STEPPER_PPR/(pi*STEPPER_PD));

%% PENDULUM DATA

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

Params = [m, l, kd, kdr, kc];

%% EXPERIMENTAL DATA

% Import data
try    
    Experimental_Data = readtable('Steps_Cart_Acc.xlsx', 'Sheet', 1);
catch
    fprintf ("Experimental data not found. Aborting \n\n");    
    return;
end 

% Get data
Time = Experimental_Data.Time;
Accel = Experimental_Data.Accel;
CurrentPPS = Experimental_Data.CurrentPPS;
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

% Estimate some of the parameters to compare
PosDotCalc = gradient(Pos(:)) ./ gradient(Time(:)); 
ThetaDotCalc = gradient(Theta(:)) ./ gradient(Time(:)); 
AccelCalc = gradient(CurrentPPS(:) / STEPPER_KV) ./ gradient(Time(:));
PPSCalc = cumtrapz(Time,Accel)*STEPPER_KV;

% Initial state vector
State0 = [Pos(1); PosDot(1); Theta(1); ThetaDot(1)];

% Clear variables
clear -regexp ^Exp;

%% SOLVE NON LINEAR MODEL

% Solve non linear model
[~, States] = ode45(@(SolverTime, State)CartPendModel(SolverTime, State, Params, Accel, Time), Time, State0);

% Get data for model
PosM = States(:,1);
PosDotM = States(:,2);
ThetaM = States(:,3);
ThetaDotM = States(:,4);

%% PLOTS

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% Input
subplot(321);
plot(Time ,Accel, 'DisplayName', 'Experimental');
hold on;
plot(Time, AccelCalc, 'DisplayName', 'Calc');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Input');
legend;

% Position
subplot(323);
plot(Time ,Pos, 'DisplayName', 'Experimental');
hold on;
plot(Time, PosM, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title('Position');
legend;

% Theta
subplot(325);
plot(Time ,Theta, 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaM, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Theta [rad]');
title('Theta');
legend;

% PPS
subplot(322);
plot(Time, CurrentPPS, 'DisplayName', 'Experimental');
hold on;
plot(Time, PPSCalc, 'DisplayName', 'Calc');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [PPS]');
title('Motor velocity');
legend;

% Velocity
subplot(324);
plot(Time, PosDot, 'DisplayName', 'Experimental');
hold on;
plot(Time, PosDotM, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocity');
legend;

% Angular velocity
subplot(326);
plot(Time, ThetaDot, 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaDotM, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Angular velocity');
legend;

%% SAVE RESULTS

if (OptSave)
    if exist('./Results/Model_Full_Validation.png', 'file')
        delete('./Results/Model_Full_Validation.png');
    end
    
    saveas(hFig, "./Results/Model_Full_Validation", 'png');  
end

%%

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);