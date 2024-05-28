%% Pendulum model - Non linear without input forces
% Renan Duarte - 30/04/2024

% States [theta thetadot]'
% m Pendulum mass kg
% l Distance to the center of gravity m
% F Friction coefficients

function dState = FreePend(state, m, l, F)
    g = 9.81; % Gravity m/s^2
    I = (m*l^2)/3; % Inertia around the center of gravity

    theta = state(1);
    thetaDot = state(2);
       
    kd = F(1);
    kdr = F(2);
    Fc = F(3);
    kt = F(4);
    
    den = m*l^2 + I;
    
    Fext = kd*thetaDot/den + (kdr*thetaDot^2)/den + Fc*tanh(kt*thetaDot)/den;
    
    dState(1, 1) = thetaDot;
    dState(2, 1) = m*g*l*sin(theta)/den - Fext;
end