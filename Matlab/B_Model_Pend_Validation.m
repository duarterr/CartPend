%% PENDULUM MODEL VALIDATION
% FREE OSCILLATIONS - NO INPUT
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
    Exp1 = readtable('20.xlsx', 'Sheet', 1);
    Exp2 = readtable('40.xlsx', 'Sheet', 1);
    Exp3 = readtable('80.xlsx', 'Sheet', 1);
catch
    fprintf ("Experimental data not found. Aborting \n\n");    
    return;
end   

% Get data for experiment 1
Time1 = Exp1.Time;
Theta1 = Exp1.Theta;
ThetaDot1 = gradient(Theta1(:)) ./ gradient(Time1(:)); 

% Get data for experiment 2
Time2 = Exp2.Time;
Theta2 = Exp2.Theta;
ThetaDot2 = gradient(Theta2(:)) ./ gradient(Time2(:)); 

% Get data for experiment 3
Time3 = Exp3.Time;
Theta3 = Exp3.Theta;
ThetaDot3 = gradient(Theta3(:)) ./ gradient(Time3(:)); 

% Initial conditions
xi1 = [0; 0; Theta1(1); ThetaDot1(1)];
xi2 = [0; 0; Theta2(1); ThetaDot2(1)];
xi3 = [0; 0; Theta3(1); ThetaDot3(1)];

% Oscillation frequencies
fs1 = PendFreq (Time1, Theta1);
fs2 = PendFreq (Time2, Theta2);
fs3 = PendFreq (Time3, Theta3);

% Clear variables
clear -regexp ^Exp;

%% NON LINEAR MODEL

% Solve non linear models
[~, States1] = ode45(@(t,y)PendCart(t, y, Time1, 0, m, l, F), Time1, xi1);
[~, States2] = ode45(@(t,y)PendCart(t, y, Time2, 0, m, l, F), Time2, xi2);
[~, States3] = ode45(@(t,y)PendCart(t, y, Time3, 0, m, l, F), Time3, xi3);

% Get data for model 1
PosM1 = States1(:,1);
PosDotM1 = States1(:,2);
ThetaM1 = States1(:,3);
ThetaDotM1 = States1(:,4);

% Get data for model 2
PosM2 = States2(:,1);
PosDotM2 = States2(:,2);
ThetaM2 = States2(:,3);
ThetaDotM2 = States2(:,4);

% Get data for model 3
PosM3 = States3(:,1);
PosDotM3 = States3(:,2);
ThetaM3 = States3(:,3);
ThetaDotM3 = States3(:,4);

% Oscillation frequencies
fsM1 = PendFreq (Time1, ThetaM1);
fsM2 = PendFreq (Time2, ThetaM2);
fsM3 = PendFreq (Time3, ThetaM3);

%% PLOTS

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% Experiment 1
subplot(311);
plot(Time1,Theta1, 'DisplayName', sprintf("Experimental: %.2fHz", fs1));
hold on;
plot(Time1, ThetaM1, 'DisplayName', sprintf("Model: %.2fHz", fsM1));
grid on;
xlim([0 Time1(end)]);
xlabel('Time (s)');
ylabel('Theta(rad)');
title(sprintf("pi + %.2f rad", xi1(3)-pi))
legend;

% Experiment 2
subplot(312);
plot(Time2,Theta2, 'DisplayName', sprintf("Experimental: %.2fHz", fs2));
hold on;
plot(Time2, ThetaM2, 'DisplayName', sprintf("Model: %.2fHz", fsM2));
grid on;
xlim([0 Time2(end)]);
xlabel('Time (s)');
ylabel('Theta(rad)');
title(sprintf("pi + %.2f rad", xi2(3)-pi))
legend;

% Experiment 3
subplot(313);
plot(Time3,Theta3, 'DisplayName', sprintf("Experimental: %.2fHz", fs3));
hold on;
plot(Time3, ThetaM3, 'DisplayName', sprintf("Model: %.2fHz", fsM3));
grid on;
xlim([0 Time3(end)]);
xlabel('Time (s)');
ylabel('Theta(rad)');
title(sprintf("pi + %.2f rad", xi3(3)-pi))
legend;

%% SAVE RESULTS

if (OptSave)
    if exist('./Results/Model_Pend_Validation.png', 'file')
        delete('./Results/Model_Pend_Validation.png');
    end
    
    saveas(hFig, "./Results/Model_Pend_Validation", 'png');  
end

%%

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);