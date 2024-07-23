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

%% EXPERIMENTAL DATA

% Import data
try    
    Exp = readtable('Debug.xlsx', 'Sheet', 1);
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
Pos = Pos-min(Pos);
PosDotCalc = gradient(Pos(:)) ./ gradient(Time(:)); 

% Initial conditions
xi = [Pos(1); PosDot(1)];

% Clear variables
clear -regexp ^Exp;

%% PLOTS

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% Input
subplot(411);
plot(Time ,Accel, 'DisplayName', 'Experimental');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Input');
legend;

% Position
subplot(412);
plot(Time ,Pos, 'DisplayName', 'Experimental');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title('Position');
legend;

% Velocity
subplot(413);
plot(Time, PosDot, 'DisplayName', 'Experimental');
hold on;
plot(Time, PosDotCalc, 'DisplayName', 'Calculated');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocity');
legend;

% PPS
subplot(414);
plot(Time, CurrentPPS, 'DisplayName', 'Experimental');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [PPS]');
title('Motor velocity');
legend;

%% MODEL

GPos = tf(1.013, [1 0.315 0]);
GPosDot = tf([0.004044 0.8799], [1 0.04475]);

[PosMod,~,~] = lsim(GPos, Accel, Time, Pos(1));
[PosDotMod,~,~] = lsim(GPosDot, Accel, Time, Pos(1));

hFig = figure(2);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);

subplot(211);
plot(Time ,Pos, 'DisplayName', 'Experimental');
hold on;
plot(Time ,PosMod, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title('Position');
legend;

subplot(212);
plot(Time ,PosDot, 'DisplayName', 'Experimental');
hold on;
plot(Time ,PosDotMod, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocity');
legend;

%% SAVE RESULTS

if (OptSave)
    if exist('./Results/Model_Cart_Estimation.png', 'file')
        delete('./Results/Model_Cart_Estimation.png');
    end
    
    saveas(hFig, "./Results/Model_Cart_Estimation", 'png');  
     
    Cart.GPos = GPos;
    Cart.GPosDot = GPosDot;

    if exist('./Results/Cart.mat', 'file')
        delete('./Results/Cart.mat');
    end
    
    save('./Results/Cart.mat', 'Cart');    

end

%%

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);