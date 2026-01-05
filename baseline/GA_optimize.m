% GA_optimize.m
% Recebe população inicial (paths cell array), mapas, wj, e parâmetros GA.
function [best_path, best_fitness] = GA_optimize(init_population, maps, ga_iters, Pc, Pm, gamma, delta)
% GA otimiza população de paths. init_population: cell array de Nx2 paths.

pop = init_population;
popN = length(pop);
best_fitness = -Inf;
best_path = [];

for gen = 1:ga_iters
    fitness = zeros(1,popN);
    for i=1:popN
        fitness(i) = evaluate_multimap(pop{i}, maps, gamma, delta);
    end
    % seleção por roleta
    idxs = roulette_selection(fitness, popN);
    newpop = cell(1,popN);
    for k = 1:2:popN
        p1 = pop{idxs(k)}; p2 = pop{idxs(min(k+1,popN))};
        if rand < Pc
            [c1,c2] = path_crossover(p1,p2);
        else
            c1 = p1; c2 = p2;
        end
        if rand < Pm, c1 = path_mutation(c1, maps{1}); end
        if rand < Pm, c2 = path_mutation(c2, maps{1}); end
        newpop{k} = c1;
        if k+1 <= popN, newpop{k+1} = c2; end
    end
    pop = newpop;
    % atualizar melhor
    [mx, ix] = max(fitness);
    if mx > best_fitness
        best_fitness = mx;
        best_path = pop{ix};
    end
    fprintf('GA gen %d best fitness=%.6f\n', gen, best_fitness);
end
end

%% --- helpers GA ---
function idxs = roulette_selection(fitness, N)
% evita negativos
minf = min(fitness);
if minf <= 0
    fitness = fitness - minf + eps;
end
probs = fitness / sum(fitness);
cum = cumsum(probs);
idxs = zeros(1,N);
for i=1:N
    r = rand;
    idxs(i) = find(cum >= r, 1, 'first');
end
end

function val = evaluate_multimap(path, maps, gamma, delta)
K = length(maps);
CM = zeros(1,K);
for j=1:K
    z = Avoid_rectangular(maps{j}, path);
    if isempty(z)
        CM(j) = 0;
    else
        f1 = 1 / (sum(sqrt(sum(diff(z).^2,2))) + eps); % fit1 (14)
        f2 = path_smoothness(z);                     % fit2 (15) -> média de ângulos
        CM(j) = gamma * f1 + delta * f2;
    end
end
% pesos subjetivos do artigo (table 1)
w = [0.1931, 0.2964, 0.5105];
val = sum(w(1:K) .* CM(1:K));
end

function s = path_smoothness(path)
f = size(path,1);
if f < 3, s = pi; return; end
angs = zeros(f-2,1);
for i=1:f-2
    p1 = path(i,:); p2 = path(i+1,:); p3 = path(i+2,:);
    a = norm(p1 - p3);
    b = norm(p1 - p2);
    c = norm(p2 - p3);
    if b==0 || c==0
        angs(i) = 0;
    else
        val = (b^2 + c^2 - a^2) / (2*b*c);
        val = max(-1,min(1,val));
        angs(i) = acos(val);
    end
end
s = mean(angs);
end

function [c1,c2] = path_crossover(p1,p2)
% corte em ponto proporcional ao comprimento
l1 = size(p1,1); l2 = size(p2,1);
if l1 < 3 || l2 < 3
    c1 = p1; c2 = p2; return;
end
pt1 = randi([2, max(2,floor(l1/2))]);
pt2 = randi([2, max(2,floor(l2/2))]);
c1 = [p1(1:pt1,:); p2(pt2+1:end,:)];
c2 = [p2(1:pt2,:); p1(pt1+1:end,:)];
end

function p = path_mutation(path, map_ref)
% mutação: remove um ponto e tenta inserir median (repair)
if size(path,1) <= 3, return; end
i1 = randi([2, size(path,1)-1]);
% tentativa de inserir ponto livre mediando
mid = round((path(i1-1,:) + path(i1+1,:))/2);
if mid(1) < 1 || mid(2) < 1 || mid(1) > size(map_ref,1) || mid(2) > size(map_ref,2)
    return;
end
if map_ref(mid(1), mid(2)) ~= 0
    path(i1,:) = mid;
else
    % escolher vizinho livre
    nbs = get_free_neighbors(map_ref, path(i1,:));
    if ~isempty(nbs)
        path(i1,:) = nbs(randi(size(nbs,1)), :);
    end
end
p = path;
end

