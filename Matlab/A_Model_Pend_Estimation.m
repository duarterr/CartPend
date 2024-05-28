%% PENDULUM MODEL ESTIMATION
% FREE OSCILLATIONS - NO INPUT
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
Experimantal_Data = readtable('20.xlsx', 'Sheet', 1);

Time = Experimantal_Data.Time;
Theta = Experimantal_Data.Theta;

% dTheta/dt
ThetaDot = gradient(Theta(:)) ./ gradient(Time(:)); 

% Frequencia do sinal experimental
fs = PendFreq (Time, Theta);

% Envelope do sinal experimental
Env = envelope(Theta);

% Initial conditions
xi = [0; 0; Theta(1); ThetaDot(1)];

%% BRUTE FORCE APPROACH - TEST ALL POSSIBLE PARAMETERS IN RANGE

% Pendulum data

% Pendulum mass kg (0.075)
mv = linspace(0.07, 0.1, 20);

% Distance to the center of gravity m (0.363)
lv = linspace(0.3, 0.45, 50); 

% Linear damping coefficient Ns/m (536e-6)
kdv =  linspace(300e-6, 600e-6, 10);

% Air drag constant Ns^2/m^2 (10e-9)
kdrv = linspace(0, 1e-8, 20);

% Coulomb constant force N (10e-6)
Fcv =  linspace(0, 1e-5, 20);

% Constant in tanh function (100)
kt = 100;

% Prepare loop
Configurations_Dimensions = [numel(mv) numel(lv) numel(kdv), numel(kdrv), numel(Fcv)];
Configurations_Max = prod(Configurations_Dimensions);

Error = NaN*ones(Configurations_Max, 1);
ppm = ParforProgressbar(Configurations_Max);

fprintf ("%d combinations will be tested \n\n", Configurations_Max);

parfor Idx_Cfg = 1:Configurations_Max 
    % Progressbar update
    ppm.increment();
    
    % Indexes of current iteration
    [Idx_m, Idx_l, Idx_kd, Idx_kdr, Idx_Fc] = ind2sub(Configurations_Dimensions, Idx_Cfg);
    
    % Current pendulum parameters
    m = mv(Idx_m);
    l = lv(Idx_l);
    kd = kdv(Idx_kd);
    kdr = kdrv(Idx_kdr);
    Fc = Fcv(Idx_Fc);
   
    % Friction array
    F = [kd kdr Fc kt]; 
    
    % Solve non linear model - No input
    [~, States] = ode45(@(t,y)PendCart(t, y, Time, 0, m, l, F), Time, xi);
    ThetaM = States(:,3);
    
    % Model oscillation frequency
    fsM = PendFreq(Time, ThetaM);
    
    % Model decay envelope
    EnvM = envelope(ThetaM);
    
    % Total error - (dif(freq) + dif(envelope)
    Error(Idx_Cfg) = abs(fsM-fs) + sum(abs(EnvM(1:end-100) - Env(1:end-100)));   
    
end
delete(ppm);

% Get result with lowest error
[Value, Idx] = min(Error(:));
[Idx_m, Idx_l, Idx_kd, Idx_kdr, Idx_Fc] = ind2sub(Configurations_Dimensions, Idx);

% Best coefficients
m = mv(Idx_m)
l = lv(Idx_l)
kd = kdv(Idx_kd)
kdr = kdrv(Idx_kdr)
Fc = Fcv(Idx_Fc)

F = [kd kdr Fc kt]; % Friction array

%% SOLVE MODEL WITH BEST PARAMETERS

% Solve non linear model
[~, States] = ode45(@(t,y)PendCart(t, y, Time, 0, m, l, F), Time, xi);

% Get states
ThetaM = States(:,3);
ThetaDotM = States(:,4);

% Model oscillating frequency
fsM = PendFreq(Time, ThetaM);

%% PLOTS

hFig = figure(1);
clf(1);
plot(Time,Theta, 'DisplayName', 'Experimental');
hold on;
plot(Time, ThetaM, 'DisplayName', 'NL model');
grid on;
xlim([0 Time(end)]);
xlabel('Time (s)');
ylabel('Theta(rad)');
legend;

%% SAVE PENDULUM OBJECT

if (OptSave)    
    Pendulum.m = m;
    Pendulum.l = l;
    Pendulum.F = F;

    if exist('./Results/Pendulum.mat', 'file')
        delete('./Results/Pendulum.mat');
    end
    save('./Results/Pendulum.mat', 'Pendulum');    

end

%%

Time_Duration = toc;
fprintf ("Calculations took %.2f seconds \n\n", Time_Duration);
