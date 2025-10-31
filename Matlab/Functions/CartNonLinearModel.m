function dState = CartRateLimiterModel(SolverTime, State, Params, Vcmd, TimeVector)
    amax = Params(1);
    tau = Params(2);
    Kv = Params(3);
    Kp = Params(4);
    
    p = State(1);
    v = State(2);
    
    if (numel(Vcmd) > 1)     
        Vcmd = Kv*interp1(TimeVector, Vcmd, SolverTime);
    end    
    
    % Velocity error
    v_error = Vcmd - v;
    
    % Simple saturation logic
    a = sign(v_error) * min(abs(v_error) / tau, amax); 
    
    dState(1) = Kp * v;
    dState(2) = a;
    
    dState = dState';
end