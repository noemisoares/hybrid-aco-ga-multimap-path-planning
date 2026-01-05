% ACO_generate_paths.m
% Executa ACO melhorado sobre ref_map e retorna conjunto de caminhos (cell array)
function paths_collected = ACO_generate_paths(map_ref, start_pos, goal_pos, m, max_iter, rho, alpha, beta, Q, maps_all)
% Implementação ACO melhorado (grid-based). Retorna cell array de paths (Nx2 arrays).

[R,C] = size(map_ref);
% inicializa tau por nó (1/lij) conforme (6)
tau = zeros(R,C);
for i=1:R
    for j=1:C
        if map_ref(i,j) == 0
            tau(i,j) = 0;
        else
            lij = sqrt((goal_pos(1)-i)^2 + (goal_pos(2)-j)^2);
            tau(i,j) = 1/(lij + eps);
        end
    end
end

paths_collected = {};
for it = 1:max_iter

    % === MODIFICAÇÃO: Heurística Adaptativa de Feromônio ===
    % Aumenta gradualmente a influência do feromônio (alpha)
    % e reduz o peso da heurística de distância (beta)
    alpha_it = alpha + (it / max_iter) * 1.5;  % mais foco em feromônio no fim
    beta_it  = beta - (it / max_iter) * 2.0;   % menos foco em distância no fim
    beta_it  = max(beta_it, 1);                % garante que beta não zere

    fprintf("Iteração ACO %d/%d | α=%.2f β=%.2f\n", it, max_iter, alpha_it, beta_it);

    all_paths = cell(1,m);
    Ls = inf(1,m);

    % === Caminhada das formigas ===
    for k = 1:m
        path = aco_single_walk(map_ref, start_pos, goal_pos, tau, alpha_it, beta_it);
        all_paths{k} = path;
        if ~isempty(path)
            Ls(k) = path_total_length(path);
        end
    end

    % === Atualização do feromônio conforme (8)-(9) com penalidade sigma (10) ===
    delta = zeros(R,C);
    for k=1:m
        path = all_paths{k};
        if isempty(path), continue; end
        Lk = Ls(k);
        for p = 1:size(path,1)-1
            i = path(p,1); j = path(p,2);
            sigma = compute_sigma(i,j,maps_all);
            delta(i,j) = delta(i,j) + (1 - sigma) * Q / (Lk + eps);
        end
    end

    % Atualiza feromônio global
    tau = (1 - rho) * tau + delta;

    % === Coleta de caminhos válidos ===
    valid_idx = find(isfinite(Ls));
    for vi = valid_idx
        if ~isempty(all_paths{vi})
            paths_collected{end+1} = all_paths{vi}; %#ok<AGROW>
        end
    end

    % Limita crescimento para economia de memória
    if length(paths_collected) > 5*m
        paths_collected = paths_collected(1:5*m);
    end
end

% === Garante número mínimo de caminhos na população ===
if isempty(paths_collected)
    paths_collected = {};
    return;
end

while length(paths_collected) < m
    idx = randi(length(paths_collected));
    p = paths_collected{idx};
    paths_collected{end+1} = mutate_path_simple(p, map_ref);
end

% Trunca se exceder
if length(paths_collected) > m
    paths_collected = paths_collected(1:m);
end
end

%% --- Funções auxiliares ACO ---
function path = aco_single_walk(map_ref, start_pos, goal_pos, tau, alpha, beta)
[R,C] = size(map_ref);
max_steps = R*C;
visited = false(R,C);
cur = start_pos;
path = cur;
visited(cur(1),cur(2)) = true;
for step = 1:max_steps
    if isequal(cur, goal_pos)
        return;
    end
    nbs = get_free_neighbors(map_ref, cur);
    % filtra não visitados
    allowed = [];
    for i=1:size(nbs,1)
        if ~visited(nbs(i,1), nbs(i,2))
            allowed = [allowed; nbs(i,:)]; %#ok<AGROW>
        end
    end
    if isempty(allowed)
        path = []; return; % dead-end
    end
    probs = zeros(size(allowed,1),1);
    for a=1:size(allowed,1)
        ni = allowed(a,1); nj = allowed(a,2);
        dij = sqrt((goal_pos(1)-ni)^2 + (goal_pos(2)-nj)^2) + eps;
        probs(a) = (tau(ni,nj)^alpha) * ((1/dij)^beta);
    end
    if sum(probs) <= 0
        idx = randi(size(allowed,1));
    else
        probs = probs / sum(probs);
        idx = roulette_select(probs);
    end
    cur = allowed(idx,:);
    path = [path; cur]; %#ok<AGROW>
    visited(cur(1),cur(2)) = true;
end
path = []; % não alcançou objetivo
end

function nbs = get_free_neighbors(map, pos)
[R,C] = size(map);
i=pos(1); j=pos(2);
dirs = [1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
nbs=[];
for d=1:size(dirs,1)
    ni=i+dirs(d,1); nj=j+dirs(d,2);
    if ni>=1 && ni<=R && nj>=1 && nj<=C
        if map(ni,nj) ~= 0
            nbs = [nbs; ni nj]; %#ok<AGROW>
        end
    end
end
end

function idx = roulette_select(probs)
r = rand();
cum = cumsum(probs);
idx = find(cum>=r,1,'first');
if isempty(idx), idx = length(probs); end
end

function L = path_total_length(path)
L=0;
for i=1:size(path,1)-1
    L = L + sqrt((path(i+1,1)-path(i,1))^2 + (path(i+1,2)-path(i,2))^2);
end
end

function sigma = compute_sigma(i,j,maps_all)
% sigma = sum_l w_l * (m_lij / n_lij)  where n_lij default 8 (neighbors)
d = length(maps_all);
% pesos subjetivos da Tabela 1
w_list = [0.1931, 0.2964, 0.5105];
sigma = 0;
for l=1:d
    mapl = maps_all{l};
    [R,C] = size(mapl);
    ml = 0; nl = 0;
    for di=-1:1
        for dj=-1:1
            if di==0 && dj==0, continue; end
            ni = i+di; nj = j+dj;
            if ni>=1 && ni<=R && nj>=1 && nj<=C
                nl = nl + 1;
                if mapl(ni,nj) == 0
                    ml = ml + 1;
                end
            end
        end
    end
    if nl==0, ratio = 0; else ratio = ml / nl; end
    sigma = sigma + w_list(min(l,length(w_list))) * ratio;
end
sigma = min(max(sigma, 0), 1); % garantir entre 0 e 1
end

function p = mutate_path_simple(path, map_ref)
% muta um nó interno para um vizinho livre (preenche população)
p = path;
if size(p,1) <= 3, return; end
idx = randi([2, size(p,1)-1]);
nbs = get_free_neighbors(map_ref, p(idx,:));
if ~isempty(nbs)
    p(idx,:) = nbs(randi(size(nbs,1)), :);
end
end

