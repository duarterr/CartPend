%% Cart-Pendulum Coupled Nonlinear Model
% Combined model with cart dynamics and inverted pendulum
% 
% Input: Vcmd - Desired velocity command
% Output: 4 states [cart position, cart velocity, pendulum angle, pendulum angular velocity]
%
% State vector:
%   State(1) = p      - Cart position [m]
%   State(2) = v      - Cart velocity [m/s]
%   State(3) = theta  - Pendulum angle [rad]
%   State(4) = thetaDot - Pendulum angular velocity [rad/s]
%
% Params vector:
%   Cart parameters:
%     Params(1) = amax - Maximum cart acceleration [m/s^2]
%     Params(2) = tau  - Time constant for velocity control [s]
%     Params(3) = Kv   - Velocity command gain
%     Params(4) = Kp   - Position derivative gain
%
%   Pendulum parameters:
%     Params(5) = m    - Pendulum mass [kg]
%     Params(6) = l    - Distance from hinge to center of gravity [m]
%     Params(7) = kd   - Linear damping coefficient [Ns/m]
%     Params(8) = kdr  - Air drag constant [Ns^2/m^2]
%     Params(9) = kc   - Coulomb friction force [N]

function dState = CartPendNonLinearModel(SolverTime, State, Params, Vcmd, TimeVector)
    %% Extract cart parameters
    amax = Params(1);
    tau = Params(2);
    Kv = Params(3);
    Kp = Params(4);
    
    %% Extract pendulum parameters
    m = Params(5);
    l = Params(6);
    kd = Params(7);
    kdr = Params(8);
    kc = Params(9);
    
    %% Extract states
    p = State(1);           % Cart position
    v = State(2);           % Cart velocity
    theta = State(3);       % Pendulum angle
    thetaDot = State(4);    % Pendulum angular velocity
    
    %% Interpolate velocity command if it's a vector
    if (numel(Vcmd) > 1)     
        Vcmd = interp1(TimeVector, Vcmd, SolverTime);
    end    
    
    %% Cart dynamics - Calculate acceleration
    v_error = Kv * Vcmd - v;
    
    % Simple saturation logic for cart acceleration
    a = sign(v_error) * min(abs(v_error) / tau, amax); 
    
    %% Pendulum dynamics
    % Gravity constant
    g = 9.81;  % [m/s^2]
    
    % Moment of inertia
    I = m * l^2;
    
    % Damping forces
    Fext = kd*thetaDot/I + (kdr*thetaDot^2)/I + kc*sign(thetaDot)/I;
    
    % Pendulum angular acceleration (with cart acceleration coupling)
    thetaDotDot = m*g*l*sin(theta)/I - m*l*cos(theta)*a/I - Fext;
    
    %% State derivatives
    dState(1) = Kp * v;           % Cart position derivative
    dState(2) = a;                % Cart velocity derivative (acceleration)
    dState(3) = thetaDot;         % Pendulum angle derivative
    dState(4) = thetaDotDot;      % Pendulum angular velocity derivative
    
    % Return as column vector
    dState = dState';
end