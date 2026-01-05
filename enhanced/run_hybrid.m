% run_hybrid.m
% Script principal para rodar o ACO-GA híbrido (implementação baseada no artigo).
% Salvem todos os .m na mesma pasta e execute este script no Octave.
clear; clc; close all;

% ---------- parâmetros ----------
m = 10;              % número de formigas / tamanho da população
aco_iters = 3;      % reduzido para teste (substitua por 90 para exec completa)
ga_iters  = 5;      % reduzido para teste (substitua por 60 para exec completa)
rho = 0.25;
alpha = 2;
beta  = 8;
Pc = 0.2;
Pm = 0.05;
Q = 100;             % constante delta tau
gamma = 0.5; delta = 0.5; % pesos fit1, fit2 (padrão neutro)

fprintf('=== Iniciando run_hybrid (versão Octave) ===\n');

% ---------- gerar mapas e mostrar ----------
maps = generate_example_maps();
figure('Name','Mapas iniciais','IntegerHandle','off');
for k=1:length(maps)
    subplot(1,3,k);
    im = zeros(size(maps{k},1),size(maps{k},2),3);
    im(:,:,1) = (maps{k}==1);      % livre -> vermelho componente
    im(:,:,2) = (maps{k}==1);      % verde para visualizar melhor
    imagesc(im); axis equal tight;
    title(sprintf('Mapa %d (0=obstáculo)',k));
end
colormap(gray);
drawnow; pause(1);

% ---------- selecionar mapa referência ----------
ref_idx = select_reference_map(maps);
ref_map = maps{ref_idx};
[start_pos, goal_pos] = find_map_start_goal(ref_map);
fprintf('Mapa referencia: %d. start=(%d,%d) goal=(%d,%d)\n', ref_idx, start_pos(1), start_pos(2), goal_pos(1), goal_pos(2));

% ---------- ACO para gerar população inicial ----------
fprintf('Executando ACO (população inicial)...\n');
paths_collected = ACO_generate_paths(ref_map, start_pos, goal_pos, m, aco_iters, rho, alpha, beta, Q, maps);

if isempty(paths_collected)
    error('ACO nao encontrou caminhos validos. Tente reduzir obstaculos ou aumentar iters.');
end
fprintf('ACO gerou %d caminhos válidos para a população inicial.\n', length(paths_collected));

% ---------- GA otimiza com fitness multi-map ----------
fprintf('Executando GA (otimizacao)...\n');
[best_path, best_fitness] = GA_optimize(paths_collected, maps, ga_iters, Pc, Pm, gamma, delta);

% ---------- plot final ----------
fprintf('Melhor fitness obtido: %.6f\n', best_fitness);
figure('Name','Melhor Caminho Final','IntegerHandle','off');
imagesc(ref_map==0); colormap(gray); axis equal tight; hold on;
plot(best_path(:,2), best_path(:,1), 'r-', 'LineWidth', 2);
plot(best_path(1,2), best_path(1,1),'ob','MarkerFaceColor','b','MarkerSize',6);
plot(best_path(end,2), best_path(end,1),'ok','MarkerFaceColor','k','MarkerSize',6);
title(sprintf('Melhor caminho no mapa %d (fitness %.6f)', ref_idx, best_fitness));
drawnow; hold off;

% save
save('hybrid_result.mat', 'best_path', 'best_fitness', 'maps');
fprintf('Resultado salvo em hybrid_result.mat\n');

