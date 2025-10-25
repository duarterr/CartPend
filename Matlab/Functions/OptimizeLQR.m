function [K_opt, Q_opt, R_opt, Results] = OptimizeLQR(Ad, Bd, Cd, Dd, Ts, ParamsOpt, xi, MaxIter)
% OPTIMIZELQR - Otimiza parâmetros Q e R para LQR discreto
%
% Inputs:
%   Ad, Bd, Cd, Dd - Matrizes do sistema discreto
%   Ts - Tempo de amostragem
%   ParamsOpt - Parâmetros do modelo não linear [m l kd kdr kc]
%   xi - Estado inicial
%   MaxIter - Número máximo de iterações (padrão: 100)
%
% Outputs:
%   K_opt - Ganho LQR ótimo
%   Q_opt - Matriz Q ótima
%   R_opt - Escalar R ótimo
%   Results - Estrutura com resultados da otimização

if nargin < 8
    MaxIter = 100;
end

fprintf('\n========================================\n');
fprintf('OTIMIZAÇÃO LQR - Busca de Parâmetros Ótimos\n');
fprintf('========================================\n\n');

% Configuração da otimização
ref = [0; 0; 0; 0];
Time = 0:Ts:10;
N = length(Time);

% Ranges de busca (em escala logarítmica para melhor cobertura)
q1_range = logspace(-1, 3, 8);  % Posição do carrinho
q2_range = logspace(-1, 1, 6);  % Velocidade do carrinho
q3_range = logspace(0, 4, 10);  % Ângulo do pêndulo (mais importante)
q4_range = logspace(-1, 2, 8);  % Velocidade angular
r_range = logspace(-1, 3, 10);   % Peso do controle

% Inicialização
best_cost = inf;
K_opt = [];
Q_opt = [];
R_opt = [];
iteration = 0;
valid_solutions = 0;

fprintf('Testando combinações de parâmetros...\n');
fprintf('Range Q1: [%.2f, %.2f]\n', min(q1_range), max(q1_range));
fprintf('Range Q3: [%.2f, %.2f]\n', min(q3_range), max(q3_range));
fprintf('Range R:  [%.2f, %.2f]\n\n', min(r_range), max(r_range));

% Grid search com amostragem aleatória
tic;
for iter = 1:MaxIter
    iteration = iteration + 1;
    
    % Seleção aleatória de parâmetros
    q1 = q1_range(randi(length(q1_range)));
    q2 = q2_range(randi(length(q2_range)));
    q3 = q3_range(randi(length(q3_range)));
    q4 = q4_range(randi(length(q4_range)));
    R = r_range(randi(length(r_range)));
    
    Q = diag([q1, q2, q3, q4]);
    
    try
        % Calcular ganho LQR
        K = dlqr(Ad, Bd, Q, R);
        
        % Verificar estabilidade em malha fechada
        poles = eig(Ad - Bd*K);
        if any(abs(poles) >= 1)
            continue;  % Sistema instável
        end
        
        % Simular resposta
        States = zeros(N, 4);
        States(1,:) = xi';
        Accel = zeros(N, 1);
        
        simulation_failed = false;
        for i = 1:N-1
            Accel(i) = K(1) * ref(1) - K * States(i,:)';
            
            % Limitar entrada (saturação)
            if abs(Accel(i)) > 50
                simulation_failed = true;
                break;
            end
            
            tspan = [Time(i), Time(i+1)];
            [~, y_temp] = ode45(@(t, y) CartPendModel(t, y, ParamsOpt, Accel(i), Time(i)), ...
                                tspan, States(i,:)');
            
            States(i+1,:) = y_temp(end,:);
            
            % Verificar divergência
            if any(abs(States(i+1,:)) > [2, 10, pi, 20])
                simulation_failed = true;
                break;
            end
        end
        
        if simulation_failed
            continue;
        end
        
        valid_solutions = valid_solutions + 1;
        
        % Calcular função de custo
        cost = CalculateCost(Time, States, Accel);
        
        % Atualizar melhor solução
        if cost < best_cost
            best_cost = cost;
            K_opt = K;
            Q_opt = Q;
            R_opt = R;
            
            fprintf('Nova melhor solução encontrada! (Iter %d)\n', iteration);
            fprintf('  Q = diag([%.2f, %.2f, %.2f, %.2f])\n', q1, q2, q3, q4);
            fprintf('  R = %.2f\n', R);
            fprintf('  Custo = %.4f\n\n', cost);
        end
        
    catch
        continue;
    end
end

elapsed = toc;

fprintf('\n========================================\n');
fprintf('OTIMIZAÇÃO CONCLUÍDA\n');
fprintf('========================================\n');
fprintf('Tempo decorrido: %.2f segundos\n', elapsed);
fprintf('Soluções válidas: %d/%d\n', valid_solutions, MaxIter);
fprintf('\nMelhores parâmetros:\n');
fprintf('  Q = diag([%.4f, %.4f, %.4f, %.4f])\n', diag(Q_opt)');
fprintf('  R = %.4f\n', R_opt);
fprintf('  Custo final = %.4f\n', best_cost);
fprintf('  K = [%.6f, %.6f, %.6f, %.6f]\n', K_opt);
fprintf('========================================\n\n');

% Armazenar resultados
Results.best_cost = best_cost;
Results.valid_solutions = valid_solutions;
Results.total_iterations = MaxIter;
Results.elapsed_time = elapsed;

end

function cost = CalculateCost(Time, States, Accel)
% Função de custo para avaliar desempenho do controlador
%
% Penalidades:
% 1. Tempo de acomodação (settling time)
% 2. Overshoot no ângulo
% 3. Energia de controle
% 4. Erro em regime permanente

settling_threshold = 0.02;  % 2% do valor inicial

% Tempo de acomodação para o ângulo (mais crítico)
theta = States(:,3);
theta_settled = find(abs(theta) < settling_threshold * abs(theta(1)), 1, 'first');
if isempty(theta_settled)
    settling_time_theta = Time(end);
else
    settling_time_theta = Time(theta_settled);
end

% Tempo de acomodação para a posição
pos = States(:,1);
pos_settled = find(abs(pos) < settling_threshold * max(abs(pos)), 1, 'first');
if isempty(pos_settled)
    settling_time_pos = Time(end);
else
    settling_time_pos = Time(pos_settled);
end

% Overshoot no ângulo
max_theta = max(abs(theta));
initial_theta = abs(theta(1));
overshoot_theta = (max_theta - initial_theta) / initial_theta;

% Energia de controle
control_energy = sum(Accel.^2);

% Erro médio quadrático
mse_theta = mean(theta.^2);
mse_pos = mean(pos.^2);

% Função de custo ponderada
w_settling_theta = 10;    % Peso para tempo de acomodação do ângulo
w_settling_pos = 5;       % Peso para tempo de acomodação da posição
w_overshoot = 20;         % Peso para overshoot
w_control = 0.001;        % Peso para energia de controle
w_mse_theta = 50;         % Peso para erro do ângulo
w_mse_pos = 1;            % Peso para erro da posição

cost = w_settling_theta * settling_time_theta + ...
       w_settling_pos * settling_time_pos + ...
       w_overshoot * overshoot_theta + ...
       w_control * control_energy + ...
       w_mse_theta * mse_theta + ...
       w_mse_pos * mse_pos;

end