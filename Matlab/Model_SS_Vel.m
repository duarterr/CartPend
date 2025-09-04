%% PENDULUM ON A CART MODEL - SPACE STATE MODEL WITH VELOCITY AS INPUT
% Renan Duarte - 21/08/2024

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
kd = Pendulum.kd;
kdr = Pendulum.kdr;
kc = Pendulum.kc;

Params = [m, l, kd, kdr, kc];

%% CART MODEL
Ac = [0 1; 0 0];
Bc = [1.115 0]';
Cc = [1 0];
Dc = 0;

[num, den] = ss2tf(Ac,Bc,Cc,Dc);
GPosU = minreal(tf(num, den))

%% PENDULUM MODEL - LINEARIZED
% FOR THETA = 0 (UP)

g = 9.81;

I = m*l^2;

a1 = kd/I;
a2 = -m*g*l/I;
b0 = 0;
b1 = m*l/I;
b2 = 0;

B0 = b0;
B1 = b1-a1*B0;
B2 = b2-a1*b1-a2*B0;

Ap = [0 1; -a2 -a1];
Bp = [B1 B2]';
Cp = [1 0];
Dp = B0;

[num, den] = ss2tf(Ap,Bp,Cp,Dp);
GThetaU = minreal(tf(num, den))

[num, den] = ss2tf(Ap,Bp,[0 1],Dp);
GX2U = minreal(tf(num, den))

%% COMPLETE SYSTEM

A = [Ac zeros(2); zeros(2) Ap];
B = [Bc; Bp];
C = [Cc zeros(1,2); zeros(1,2) Cp];
D = [Dc; Dp];

%% LQR DESIGN

Q = [5 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 1];

R = 0.1;

K = lqr(A,B,Q,R)















%% 

Time = linspace (0,100,10000);
Accel = 1*(Time > 1 & Time <= 2) + -2*(Time > 2 & Time <= 2.5);
Vel = cumtrapz(Time,Accel);
State0 = [0; 0; pi; 0];

%% SOLVE NON LINEAR MODEL

% Solve non linear model
[~, States] = ode45(@(SolverTime, State)CartPendModel(SolverTime, State, Params, Accel, Time), Time, State0);

% Get data for model
PosM = States(:,1);
PosDotM = States(:,2);
ThetaM = States(:,3);
ThetaDotM = States(:,4);

%%

Sys = ss(Ap,Bp,Cp,Dp);
y = lsim(GThetaU, Vel, Time, pi); 

% Get data for model
ThetaML =y;



%% PLOTS

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% Input
subplot(321);
plot(Time ,Accel, 'DisplayName', 'Experimental');
% hold on;
% plot(Time, AccelCalc, 'DisplayName', 'Calc');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Input');
legend;

% Position
subplot(323);
% plot(Time ,Pos, 'DisplayName', 'Experimental');
% hold on;
plot(Time, PosM, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Position [m]');
title('Position');
legend;

% Theta
subplot(325);
plot(Time, ThetaM, 'DisplayName', 'Non Linear Model');
hold on;
plot(Time ,ThetaML, 'DisplayName', 'Linear Model');

grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Theta [rad]');
title('Theta');
legend;

% Velocity
subplot(324);
plot(Time, PosDotM, 'DisplayName', 'Non Linear Model');
hold on;
plot(Time, Vel, 'DisplayName', 'Linear Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocity');
legend;

% Angular velocity
subplot(326);
% plot(Time, ThetaDot, 'DisplayName', 'Experimental');
% hold on;
plot(Time, ThetaDotM, 'DisplayName', 'Model');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Angular velocity');
legend;

%%

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);

