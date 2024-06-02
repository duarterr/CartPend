%% PENDULUM ON A CART MODEL - LINEARIZATION
% Renan Duarte - 02/05/2024

format long eng;
clear all;
%close all;
clc;

% Search stuff also in this folder
addpath ('./Datasources/');
addpath ('./Functions/');
addpath ('./Results/');

% Start counting time
tic;

%% SYSTEM MODELS

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
PosDotCalc = gradient(Pos(:)) ./ gradient(Time(:)); 
Theta = Exp.Theta;
ThetaDot = Exp.ThetaDot;
ThetaDotCalc = gradient(Theta(:)) ./ gradient(Time(:)); 

PPSCalc = cumtrapz(Time,Accel)*160000;

% Initial conditions
xi = [Pos(1); PosDot(1); Theta(1); ThetaDot(1)];

% Final time
tfinal = Time(end);

% Clear variables
clear -regexp ^Exp;

%% CART MODEL - ALREADY LINEAR
Ac = [0 1; 0 0];
Bc = [0 1]';
Cc = [1 0];
Dc = 0;

[num, den] = ss2tf(Ac,Bc,Cc,Dc);
GPosU = tf(num, den)

%% PENDULUM MODEL - LINEARIZED
% FOR THETA = 0 (UP)

Kd = 3*F(1); % Maybe need to be increased
I = (m*l^2)/3;
g = 9.81;
den = m*l^2 + I;
a = m*g*l/den;
b = -Kd/den;
c = -m*l/den;

Ap = [0 1; a b];
Bp = [0 c]';
Cp = [1 0];
Dp = 0;

[num, den] = ss2tf(Ap,Bp,Cp,Dp);
GThetaU = tf(num, den)

%% COMPLETE SYSTEM

A = [Ac zeros(2); zeros(2) Ap];
B = [Bc; Bp];
C = [Cc zeros(1,2); zeros(1,2) Cp];
D = [Dc; Dp];

%% DISCRETE SYSTEM

% Sampling time
Ts = 1/200;

[Ad Bd Cd Dd] = ssdata(c2d(ss(A,B,C,D), Ts));

%% LQR DESIGN

Q = [25 0 0 0;
    0 1 0 0;
    0 0 5 0;
    0 0 0 0.1];

R = 1;

K = lqr(A,B,Q,R)

%% PERFORMANCE EVALUATION

TimeM = 0:Ts:tfinal;
ref = [0; 0; 0; 0];

% Solve non linear model
[~, States] = ode45(@(t,y)PendCart(t, y, TimeM, -K*(y-ref), m, l, F), TimeM, xi);

% Get data for model
PosM = States(:,1);
PosDotM = States(:,2);
ThetaM = States(:,3);
ThetaDotM = States(:,4);

% Estimate acceleration input
AccelM = gradient(PosDotM(:)) ./ gradient(TimeM(:)); 

%% PLOTS

% Define the colors
color1 = [0, 0.4470, 0.7410];
color2 = [0.8500, 0.3250, 0.0980];

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% Input
subplot(311);
hold on;
plot(Time, Accel, 'DisplayName', 'Experimental');
plot(TimeM, AccelM, '--', 'DisplayName', 'Model');
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Input');
grid on;
legend;

% Position
subplot(312);
hold on;

plot(Time, Pos, 'Color', color1, 'DisplayName', 'Experimental');
plot(TimeM, PosM, '--', 'Color', color1, 'DisplayName', 'Model');
ylabel('Position [m]');

yyaxis right
plot(Time, PosDot, 'Color', color2, 'DisplayName', 'Experimental');
plot(TimeM, PosDotM, '--', 'Color', color2, 'DisplayName', 'Model');
ylabel('Velocity [m/s]');

xlim([0 Time(end)]);
xlabel('Time [s]');
title('Cart');
grid on;
legend;

% Theta
subplot(313);
hold on;

plot(Time, Theta, 'Color', color1, 'DisplayName', 'Experimental');
plot(TimeM, ThetaM, '--', 'Color', color1, 'DisplayName', 'Model');
ylabel('Theta [rad]');

yyaxis right
plot(Time, ThetaDot, 'Color', color2, 'DisplayName', 'Experimental');
plot(TimeM, ThetaDotM, '--', 'Color', color2, 'DisplayName', 'Model');
ylabel('Angular Velocity [rad/s]');

xlim([0 Time(end)]);
xlabel('Time [s]');
title('Pendulum');
grid on;
legend;