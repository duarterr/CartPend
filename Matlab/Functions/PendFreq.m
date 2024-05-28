%% CALCULATE PENDULUM OSCILLATION FREQUENCY BASED ON TIMExTHETA DATA

function fo = PendFreq (Time, Theta)
    Nfft = numel(Theta);
    fsample = 1/mean(diff(Time));
    [P, f] = pwelch(Theta-pi, gausswin(Nfft),Nfft/2, Nfft, fsample);
    [~,loc] = max(P);
    fo = f(loc);
end