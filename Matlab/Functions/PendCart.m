%% Pendulum model - Non linear
% Renan Duarte - 30/04/2024

% t -> Current solver time
% state -> [pos posDot theta thetadot]'
% timeSpan -> Time vector of input u
% u -> Input vector - Cart acceleration (m/s^2)
% m -> Pendulum mass kg
% l -> Distance to the center of gravity m
% F -> Damper coefficients

function dState = PendCart(t, state, timeSpan, u, m, l, F)
    g = 9.81; % Gravity m/s^2
    I = (m*l^2)/3; % Inertia around the center of gravity

    % Damper coefficients
    kd = F(1);
    kdr = F(2);
    Fc = F(3);
    kt = F(4);
    
    % Has input vector
    if (numel(u) > 1)       
        u = interp1(timeSpan, u, t);
    end    
        
    % Get states
    pos = state(1);
    posDot = state(2);
    theta = state(3);
    thetaDot = state(4);     
    
    % Denominator of equations
    den = m*l^2 + I;
    
    % Damper 
    Fext = kd*thetaDot/den + (kdr*thetaDot^2)/den + Fc*tanh(kt*thetaDot)/den;
    
    dState(1, 1) = posDot;
    dState(2, 1) = u;
    dState(3, 1) = thetaDot;
    dState(4, 1) = m*g*l*sin(theta)/den - m*l*cos(theta)*u/den - Fext;
end