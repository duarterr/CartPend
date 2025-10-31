%% Pendulum model - Non linear
% Renan Duarte - 16/08/2024

% Params -> [m l I kd kdr Fc kt]

% m -> Pendulum mass [kg]
% l -> Distance from hinge point to the center of gravity [m]
% kd -> Linear damping coefficient [Ns/m] 
% kdr -> Air drag constant [Ns^2/m^2]
% kc -> Coulomb constant force [N] 

function dState = PendNonLinearModel(SolverTime, State, Params, Accel, TimeVector)
    % Pendulum parameters
    m = Params(1);
    l = Params(2);
    
    % Damper coefficients
    kd = Params(3);
    kdr = Params(4);
    kc = Params(5);
    
    % Gravity m/s^2    
    g = 9.81; 
    
    % Moment of inertia
    I = m*l^2;
    
    % Get states
    theta = State(1);
    thetaDot = State(2);     
       
    % Has input vector
    if (numel(Accel) > 1)     
        % Interpolate input Accel to the current solver time
        Accel = interp1(TimeVector, Accel, SolverTime);
    end    
    
    % Damper 
    Fext = kd*thetaDot/I + (kdr*thetaDot^2)/I + kc*sign(thetaDot)/I;
    
    % Non linear EDOs
    dState(1) = thetaDot;
    dState(2) = m*g*l*sin(theta)/I - m*l*cos(theta)*Accel/I - Fext;
    
    dState = dState';
end