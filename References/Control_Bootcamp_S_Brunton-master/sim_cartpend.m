clear all, close all, clc

m = 1;
M = 2;
L = 1;
g = -9.8;
d = 10;

tspan = 0:.1:50;
y0 = [0; 0; deg2rad(179); 0];
[t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);

figure(1);
pause(1);
subplot(121)
for k=1:length(t)
    drawcartpend(y(k,:),m,M,L);
end

% function dy = pendcart(y,m,M,L,g,d,u)