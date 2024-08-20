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
kd = Pendulum.kd;
kdr = Pendulum.kdr;
kc = Pendulum.kc;

%% CART MODEL - ALREADY LINEAR
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
a = m*g*l/I;
b = -kd/I;
c = -m*l/I;

Ap = [0 1; a b];
Bp = [-1/l 0]';
Cp = [1 0];
Dp = 0;

[num, den] = ss2tf(Ap,Bp,Cp,Dp);
GThetaU = minreal(tf(num, den))

%% COMPLETE SYSTEM

A = [Ac zeros(2); zeros(2) Ap];
B = [Bc; Bp];
C = [Cc zeros(1,2); zeros(1,2) Cp];
D = [Dc; Dp];

%% LQR DESIGN

Q = [5 0 0 0;
    0 0 0 0;
    0 0 10 0;
    0 0 0 0];

R = 0.1;

K = lqr(A,B,Q,R)

%% PERFORMANCE EVALUATION

Ts = 1/200;
Time = 0:Ts:5;
xi = [0.2; 0; -0.2; 0];
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
fprintf (".K = {%.6f, %.6f, %.6f, %.6f}, \\ \n", K(1), K(2), K(3), K(4));

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
    PendCartDraw([PosM(k) ThetaM(k)], Time(k), 'Response');
    %pause(Time(k+1)-Time(k));
end


%% SAVE RESULTS

if (OptSave)
    if exist('./Results/LQR_Response.png', 'file')
        delete('./Results/LQR_Response.png');
    end
    
    saveas(hFig, "./Results/LQR_Response", 'png');  
end