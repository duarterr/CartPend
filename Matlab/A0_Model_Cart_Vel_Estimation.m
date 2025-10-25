%% CART MODEL ESTIMATION - VELOCITY AS INPUT
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
    Exp = readtable('A0_Model_Cart_Vel_Estimation.xlsx', 'Sheet', 1);
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
AccelCalc = gradient(PosDotCalc(:)) ./ gradient(Time(:)); 

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

% Position
subplot(413);
plot(Time ,Pos, 'DisplayName', 'Experimental');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title('Position');
legend;

% Input
subplot(414);
plot(Time ,Accel, 'DisplayName', 'Experimental');
hold on;
plot(Time, AccelCalc, 'DisplayName', 'Calculated');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Acceleration');
legend;

%% MODEL

% Data object
Ts = mean(diff(Time));
DataPos = iddata(Pos, PosDot, Ts, 'OutputName', 'Position', 'InputName', 'Velocity');

GPosVel = tf(procest(DataPos,'P0I'))
GPosDotVel = tf(1)

[PosMod,~,~] = lsim(GPosVel, PosDot, Time);
[PosDotMod,~,~] = lsim(GPosDotVel, PosDot, Time);

PosMod = PosMod + Pos(1);

hFig = figure(2);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);

subplot(211);
plot(Time, Pos, 'DisplayName', 'Experimental');
hold on;
plot(Time, PosMod, 'DisplayName', 'Model');
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
    if exist('./Results/A0_Model_Cart_Vel_Estimation.png', 'file')
        delete('./Results/A0_Model_Cart_Vel_Estimation.png');
    end
    
    saveas(hFig, "./Results/A0_Model_Cart_Vel_Estimation", 'png');  
     
    Cart.GPos = GPosVel;
    Cart.GPosDot = GPosDotVel;

    if exist('./Results/Model_Cart_Vel.mat', 'file')
        delete('./Results/Model_Cart_Vel.mat');
    end
    
    save('./Results/Model_Cart_Vel.mat', 'GPosVel');    

end

%%

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);