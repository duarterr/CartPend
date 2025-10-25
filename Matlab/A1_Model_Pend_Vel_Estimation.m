%% PENDULUM MODEL ESTIMATION - VELOCITY AS INPUT
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
CurrentPPS = Exp.CurrentPPS;

Pos = Exp.Pos;
PosDot = Exp.PosDot;
Theta = Exp.Theta;
ThetaDot = Exp.ThetaDot;

PosDotCalc = gradient(Pos(:)) ./ gradient(Time(:)); 
ThetaDotCalc = gradient(Theta(:)) ./ gradient(Time(:)); 

% Clear variables
clear -regexp ^Exp;

%% PLOTS

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% Velocity
subplot(411);
plot(Time, PosDot, 'DisplayName', 'Experimental');
hold on;
plot(Time, PosDotCalc, 'DisplayName', 'Calculated');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Cart velocity');
legend;

% PPS
subplot(412);
plot(Time, CurrentPPS, 'DisplayName', 'Experimental');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [PPS]');
title('Motor velocity');
legend;

% Theta
subplot(413);
plot(Time, Theta, 'DisplayName', 'Experimental');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angle [rad]');
title('Theta');
legend;

% Input
subplot(414);
plot(Time, ThetaDot, 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaDotCalc, 'DisplayName', 'Calculated');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
title('Angular velocity');
legend;

%% MODEL

% Data object
Ts = mean(diff(Time));
DataTheta = iddata(Theta-pi, PosDot, Ts, 'OutputName', 'Theta', 'InputName', 'Velocity');
DataThetaDot = iddata(ThetaDot, PosDot, Ts, 'OutputName', 'ThetaDot', 'InputName', 'Velocity');

GThetaVel = tfest(DataTheta, 2, 2)
GThetaDotVel = tfest(DataTheta, 3, 3)

[ThetaMod,~,~] = lsim(GThetaVel, PosDot, Time);
[ThetaDotMod,~,~] = lsim(GThetaDotVel, PosDot, Time);

ThetaMod = ThetaMod + Theta(1);

hFig = figure(2);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);

subplot(211);
plot(Time, Theta, 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaMod, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Theta [rad]');
title('Angle');
legend;

subplot(212);
plot(Time, ThetaDot, 'DisplayName', 'Experimental');
hold on;
plot(Time ,ThetaDotMod, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Velocity');
legend;

%% SAVE RESULTS

if (OptSave)
    if exist('./Results/A1_Model_Pend_Vel_Estimation.png', 'file')
        delete('./Results/A1_Model_Pend_Vel_Estimation.png');
    end
    
    saveas(hFig, "./Results/A1_Model_Pend_Vel_Estimation", 'png');  
     
    Cart.GPos = GThetaVel;
    Cart.GPosDot = GThetaDotVel;

    if exist('./Results/Model_Pend_Vel.mat', 'file')
        delete('./Results/Model_Pend_Vel.mat');
    end
    
    save('./Results/Model_Pend_Vel.mat', 'GThetaVel');    

end

%%

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);