%% PENDULUM ON A CART MODEL - LINEARIZATION
% Renan Duarte - 02/05/2024

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
Bc = [0 1]';
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

% Sample time
Ts = 1/200;

% Discretization
sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

%% AUTOMATIC LQR DESIGN

% % Otimização automática
% MaxIter = 300;
% 
% xi = [0; 0; 0.1; 0];
% ParamsOpt = [m l kd kdr kc];
% 
% [K, Q, R, OptResults] = OptimizeLQR(Ad, Bd, Cd, Dd, Ts, ParamsOpt, xi, MaxIter);
%         
% fprintf('\nGanhos LQR finais:\n');
% fprintf('K = [%.6f, %.6f, %.6f, %.6f]\n', K(1), K(2), K(3), K(4));

%% LQR DESIGN
    
Q = diag([1, 0, 200, 30])
R = 0.1;

K = dlqr(Ad, Bd, Q, R)
 
K = [-2, -10, -40, -20]

%% POLES LOCATIONS

p = eig(Ad);

fprintf ("Open loop poles (z):\n");
display(p);

p = eig(Ad - Bd*K);

fprintf ("Closed loop poles (z):\n");
display(p);

%% PERFORMANCE EVALUATION

Time = 0:Ts:10;
xi = [0; 0; -0.1; 0];
ref = [0; 0; 0; 0];
N = length(Time);

States = zeros(N, 4);
States(1,:) = xi';
Accel = zeros(N, 1);

% Parâmetros do modelo não linear
ParamsOpt = [m l kd kdr kc];

% Simulação com ZOH (entrada constante entre amostras)
for i = 1:N-1
    % Controle LQR discreto
    Accel(i) = K(1) * ref(1) - K * States(i,:)';
    
    % Simular modelo NÃO LINEAR durante Ts com entrada constante
    tspan = [Time(i), Time(i+1)];
    [~, y_temp] = ode45(@(t, y) CartPendModel(t, y, ParamsOpt, Accel(i), Time(i)), ...
                        tspan, States(i,:)');
    
    % Próximo estado = final da integração
    States(i+1,:) = y_temp(end,:);
end

% Get data for model
PosM = States(:,1);
PosDotM = States(:,2);
ThetaM = States(:,3);
ThetaDotM = States(:,4);

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);
fprintf ("float Gains[4] = {%.6f, %.6f, %.6f, %.6f}; \n", K(1), K(2), K(3), K(4));

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

return

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