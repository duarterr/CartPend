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

% Dialog popup - Save results
OptSave = SavePopUp;

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

%% EXTENDED SYSTEM - NEW STATE w IS THE INTEGTRAL OF xRef - xPos

A1 = [A(1,:) 0;
    A(2,:) 0;
    A(3,:) 0;
    A(4,:) 0;
    -1 0 0 0 0];

B1 = [B; 0];

%% LQR DESIGN

% +-: 1 0.1 0.5 0.01 1 % Normal system

Q = [1 0 0 0;
    0 0.1 0 0;
    0 0 0.5 0;
    0 0 0 0.01];

R = 1;

Q1 = [10 0 0 0 0;
    0 1 0 0 0;
    0 0 0.1 0 0;
    0 0 0 0.5 0;
    0 0 0 0 0.01];

R1 = 1;

K = lqr(A,B,Q,R)
K1 = lqr(A1,B1,Q1,R1)

%% PERFORMANCE EVALUATION

Time = 0:.1:50;
xi = [-0.2; 0; 0; 0];
ref = [0; 0; 0; 0];

% Solve non linear model
[~, States] = ode45(@(t,y)PendCart(t, y, Time, -K*(y-ref), m, l, F), Time, xi);

% Get data for model
PosM = States(:,1);
PosDotM = States(:,2);
ThetaM = States(:,3);
ThetaDotM = States(:,4);

% Estimate acceleration input
Accel = gradient(PosDotM(:)) ./ gradient(Time(:)); 

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);
fprintf (".K = {%.6f, %.6f, %.6f, %.6f}\n", K(1), K(2), K(3), K(4));

%% PLOTS

hFig = figure(1);
set(hFig, 'units', 'normalized', 'InnerPosition',[0 0 1 1]);
clf(1);

% Input
subplot(321);
plot(Time, Accel, 'DisplayName', 'Input');
grid on;
xlim([0 Time(end)]);
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Input');
%legend;

% Position
subplot(323);
plot(Time, PosM, 'DisplayName', 'Model');
grid on;
ylabel('Position [m]');
yyaxis right
plot(Time, PosDotM, 'DisplayName', 'Model');
ylabel('Velocity [m/s]');
xlim([0 Time(end)]);
xlabel('Time [s]');
title('Cart');
%legend;

% Theta
subplot(325);
plot(Time, ThetaM, 'DisplayName', 'Model');
grid on;
ylabel('Theta [rad]');
yyaxis right
plot(Time, ThetaDotM, 'DisplayName', 'Model');
ylabel('Angular Velocity [rad/s]');
xlim([0 Time(end)]);
xlabel('Time [s]');
title('Pendulum');
%legend;

% Cart animation
subplot(3,2,[2 4 6]);
for k=1:10:length(Time)
    PendCartDraw([PosM(k) PosDotM(k) ThetaM(k) ThetaDotM(k)], Time(k), 'Response');
    %pause(Time(k+1)-Time(k));
end


%% SAVE RESULTS

if (OptSave)
    if exist('./Results/LQR_Response.png', 'file')
        delete('./Results/LQR_Response.png');
    end
    
    saveas(hFig, "./Results/LQR_Response", 'png');  
end