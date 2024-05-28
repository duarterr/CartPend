function PendCartDraw(y, t, plotTitle)
x = y(1);
th = y(3);

% dimensions
L = 0.5; % Pendulum length
W = 0.04;  % cart width
H = 0.05; % cart height
wr = .01; % wheel radius
mr = 0.03; % mass radius

% positions
% y = wr/2; % cart vertical position
y = wr/2+H/2; % cart vertical position
w1x = x-.9*W/2;
w1y = 0;
w2x = x+.9*W/2-wr;
w2y = 0;

px = x - L*sin(th);
py = y + L*cos(th);

% Trilho
plot([-10 10],[0 0],'k','LineWidth',2)
hold on

% Cart
rectangle('Position', [x-W/2, y-H/2, W, H],'Curvature',.1,'FaceColor',[1 0.1 0.1])

% Haste
plot([x px],[y py],'k','LineWidth',2)

% Massa pendulo
rectangle('Position',[px-mr/2, py-mr/2, mr, mr*2],'Curvature',1,'FaceColor',[.1 0.1 1])

% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([-0.3 0.3]);
ylim([-0.6 0.6]);
title (sprintf("%s - %.2fs", plotTitle, t));
grid on;
%set(gcf,'Position',[100 550 1000 400])
% box off
drawnow
hold off