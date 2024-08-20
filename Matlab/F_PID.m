%% CART PEND CONTROLLER DESIGN
% Renan Duarte - 11/08/2024

clear all
close all
format long eng
clc

Bode_Opt = bodeoptions;
Bode_Opt.FreqUnits = 'Hz';
Bode_Opt.Grid = 'on';
%Bode_Opt.PhaseWrapping = 'on';


%% SYSTEM MODELS

% Import data
try
    load('./Results/Cart.mat');    
catch
    fprintf ("Cart data not found. Aborting \n\n");    
    return;
end

GPos = Cart.GPos;

%% SYSTEM CHARACTERISTICS

fs = 500;       % Sampling frequency
Ts = 1/fs;      % Sampling period
fc = 200000;    % Filter cutout frequency

s = zpk('s');
z = zpk('z', Ts);

%% SYSTEM MODEL

% Gp (s domain)
Gps = GPos;

% Filter
Hs = 1;%2*pi*fc /(s + 2*pi*fc);

%% CONTROLLER

% GpH(s) - System + Filter - s domain
GpHs = Gps*Hs;

% GpH'(z) - z domain - With unit delay included
GpHz = c2d (GpHs, Ts, 'zoh')*(z^-1)

% GpH(w) - w domain
GpHw = d2c (GpHz, 'tustin');

% Gc(w) - w domain
Gcw = zpk(116.04*tf([1 0.0006049],[1 1882e-7]))

% Conversion to z domain - Including the unit delay that was in GpHz
Gcz = c2d (Gcw, Ts, 'tustin')*(z^-1)

[num, den] = tfdata(Gcz);
num = cell2mat(num)
den = cell2mat(den)

% Backwards way - Starting from experimental
Gcz_experimental = (116.040048*z - 116.039908) / (z^2 - 1*z);
Gcw_experimental = minreal(d2c(Gcz_experimental/(z^-1), 'tustin'))

%% PERFORMANCE ANALISYS

% Gp(z) - z domain - Without unit delay Z^-1
Gpz = c2d (Gps, Ts, 'zoh');

% H(z) - z domain
Hz = 1;%c2d (Hs, Ts, 'tustin');

hFig = figure(1);
subplot(131);

% Open loop Bode plot - s, z and w domains
bode (GpHs, GpHz, GpHw, Bode_Opt)
lgd = legend ('GpH(s)', 'GpH(z)', 'GpH(w)');
lgd.Location = 'best';

subplot(132);

% Open loop Bode plot with controller
bode(Gpz, Gpz*Gcz*Hz, Bode_Opt);
lgd = legend ('Não compensada', 'Compensada');
lgd.Location = 'best';

subplot(133);

% Closed loop step response
h = stepplot(feedback (Gpz*Gcz,Hz));
h.showCharacteristic('PeakResponse');
h.showCharacteristic('SettlingTime');