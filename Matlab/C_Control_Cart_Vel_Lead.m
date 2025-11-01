%% CART CONTROLLER DESIGN
% Renan Duarte - 11/08/2024

clear all
close all
format long
clc

Bode_Opt = bodeoptions;
Bode_Opt.FreqUnits = 'Hz';
Bode_Opt.Grid = 'on';
%Bode_Opt.PhaseWrapping = 'on';

%% SYSTEM CHARACTERISTICS

fs = 50;        % Sampling frequency
Ts = 1/fs;      % Sampling period

s = zpk('s');
z = zpk('z', Ts);

%% CART NON LINEAR MODEL

% Import data
try
    load('./Results/Cart.mat');    
catch
    fprintf ("Cart data not found. Aborting \n\n");    
    return;
end

amax = Cart.Amax;
Tr = Cart.Tr;
Kv = Cart.Kv;
Kp = Cart.Kp;

%% CART MODEL

% State-space representation
Ac = [0,         Kp;
      0,    -1/Tr];

Bc = [0;
      Kv/Tr];

Cc = [1 0];
Dc = 0;

% % Gp (s domain) - TF Pos/VelCmd
[num, den] = ss2tf(Ac, Bc, Cc, Dc);
Gps = tf(num, den);

% Gp(z) - z domain
Gpz = c2d (Gps, Ts, 'zoh')

%% CONTROLLER

% Gc(z) - z domain
Gcz = -0.11427*(z - 4.982)/(z-0.9323);

[num, den] = tfdata(Gcz, 'v');

A = num(1)/den(1)
B = num(2)/den(1)
C = den(2)/den(1)

%% PERFORMANCE ANALISYS

hFig = figure(1);
subplot(131);

% Open loop Bode plot - s, z and w domains
bode (Gps, Gpz, Bode_Opt)
lgd = legend ('Gp(s)', 'Gp(z)');
lgd.Location = 'best';

subplot(132);

% Open loop Bode plot with controller
bode(Gpz, Gpz*Gcz, Bode_Opt);
lgd = legend ('Não compensada', 'Compensada');
lgd.Location = 'best';

subplot(133);

% Closed loop step response
t = 0:Ts:5;
Ref = 0.1*ones(size(t));

Gmfz_r2y = feedback (Gpz*Gcz,1);
Gmfz_r2u = feedback (Gcz, Gpz);

y = lsim(Gmfz_r2y, Ref, t);
u = lsim(Gmfz_r2u, Ref, t);

subplot(133);
plot (t, y, 'DisplayName', 'Y');
ylabel('Position [m]');
yyaxis right
plot (t, u, 'DisplayName', 'U');
ylabel('Velocity [m/s]');
grid on;
xlabel('Time [s]');
title('Step response');
legend;


