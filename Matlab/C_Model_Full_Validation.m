%% PENDULUM MODEL VALIDATION
% Renan Duarte - 30/04/2024

format long eng;
clear all;
close all;
clc;

% Search stuff also in this folder
addpath ('./Datasources/');
addpath ('./Functions/');
addpath ('./Results/');

% Dialog popup - Save results
OptSave = SavePopUp;

% Start counting time
tic;

%%

STEPPER_STEPS_REV = 200;
STEPPER_MICROSTEPS = 32;
STEPPER_PPR = (STEPPER_STEPS_REV*STEPPER_MICROSTEPS);
STEPPER_PD = 0.0136;
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
F = Pendulum.F;

%% EXPERIMENTAL DATA

% Import data
try    
    Exp = readtable('Steps_Cart_Acc.xlsx', 'Sheet', 1);
catch
    fprintf ("Experimental data not found. Aborting \n\n");    
    return;
end 

% Get data
Time = Exp.Time;
Accel = Exp.Accel;
CurrentPPS = Exp.CurrentPPS;
Pos = Exp.Pos;
PosDot = Exp.PosDot;
PosDotCalc = gradient(Pos(:)) ./ gradient(Time(:)); 
Theta = Exp.Theta;
ThetaDot = Exp.ThetaDot;
ThetaDotCalc = gradient(Theta(:)) ./ gradient(Time(:)); 

AccelCalc = gradient(CurrentPPS(:) / STEPPER_KV) ./ gradient(Time(:));
PPSCalc = cumtrapz(Time,Accel)*STEPPER_KV;

% Initial conditions
xi = [Pos(1); PosDot(1); Theta(1); ThetaDot(1)];

% Clear variables
clear -regexp ^Exp;

%% NON LINEAR MODEL

% Solve non linear model
[~, States] = ode45(@(t,y)PendCart(t, y, Time, Accel, m, l, F), Time, xi);

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
plot(Time, AccelCalc, 'DisplayName', 'Model');
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