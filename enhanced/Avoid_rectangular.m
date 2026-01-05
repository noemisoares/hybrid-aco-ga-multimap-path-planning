function final_path = Avoid_rectangular(M, path)
% Implementacao do Algorithm 1 do artigo (retangular expansion + detour BFS)
% M: mapa (0 obst, 1 livre), path: Nx2 (row,col)
final_path = path;
% passo 1: achar todos os pontos da path que caem em obstáculo no mapa M
obs_idx = find(M(sub2ind(size(M), path(:,1), path(:,2))) == 0);
if isempty(obs_idx), return; end

i_cur = 1;
detour_segments = {};
while i_cur <= length(obs_idx)
    idx = obs_idx(i_cur);
    % inicia regiao retangular com esse nó
    r = path(idx,1); c = path(idx,2);
    [rmin,rmax,cmin,cmax] = rect_init_expand(M, r, c);
    found_detour = false;
    % tentar expandir até encontrar detour
    while ~found_detour
        % obter pontos de entrada/saida da regiao (prox livre antes e depois)
        % localizar ponto anterior livre da path (antes do primeiro obst no rect)
        first_on_rect = find_in_path_region(path, rmin, rmax, cmin, cmax, 'first');
        last_on_rect  = find_in_path_region(path, rmin, rmax, cmin, cmax, 'last');
        % pontos imediatamente antes e depois na path (se existirem)
        if first_on_rect > 1
            p_in = path(first_on_rect-1,:);
        else
            p_in = nearest_free(M, path(first_on_rect,:));
        end
        if last_on_rect < size(path,1)
            p_out = path(last_on_rect+1,:);
        else
            p_out = nearest_free(M, path(last_on_rect,:));
        end
        % procurar detour nas duas bordas (top-bottom / left-right)
        % cria grid com obstaculos = interior do rect (1->obst) para BFS
        mask = zeros(size(M));
        mask(rmin:rmax, cmin:cmax) = (M(rmin:rmax, cmin:cmax) == 0);
        % permutar bordas: tentar contornar pela borda externa (expandir passagem)
        % BFS no mapa original M, mas proibindo entrar na area retangular (marca como obst)
        Mtemp = M;
        Mtemp(rmin:rmax, cmin:cmax) = 0; % tratar a regiao como obst
        path1 = bfs_grid_with_mask(Mtemp, p_in, p_out); % tentativa 1
        % tentativa 2: expandir retangulo por 1 em cada direcao e tentar novamente
        if isempty(path1)
            % expandir retangulo se possível
            [rmin2,rmax2,cmin2,cmax2, expanded] = rect_expand_one(M, rmin, rmax, cmin, cmax);
            if ~expanded
                % sem expansão possivel -> nao encontrou detour
                break;
            end
            rmin=rmin2; rmax=rmax2; cmin=cmin2; cmax=cmax2;
            % loop continuará e tentará novo BFS
        else
            found_detour = true;
            detour = path1;
            detour_segments{end+1} = detour; %#ok<AGROW>
            % pular obs_idx até (last_on_rect)
            % encontrar índice de path onde last_on_rect aparece entre obs_idx
            % avançar i_cur até o último obstáculo dentro este rect
            advance_index = find(obs_idx <= last_on_rect, 1, 'last');
            if isempty(advance_index)
                i_cur = i_cur + 1;
            else
                i_cur = advance_index + 1;
            end
        end
    end
    if ~found_detour
        % não encontrou detour: tentar tratar cada obstáculo individual e continuar
        i_cur = i_cur + 1;
    end
end

% combine detour segments com path original:
if isempty(detour_segments)
    final_path = path;
    return;
end
% Reconstruir final_path substituindo sequências por detours (simplificado)
% percorre original path e quando encontra nó que é começo de alguma detour
fp = [];
i=1;
while i <= size(path,1)
    % checar se ponto atual é o início de alguma detour (com proximidade)
    added = false;
    for s=1:length(detour_segments)
        ds = detour_segments{s};
        % se path(i) é vizinho do primeiro ponto de ds, inserir ds then skip until last near ds
        if norm(path(i,:) - ds(1,:)) <= 1.5
            fp = [fp; ds]; %#ok<AGROW>
            % pular até próximo ponto da path que é vizinho do último ponto de ds
            % encontrar j tal que norm(path(j,:) - ds(end,:)) <=1.5
            j = i;
            while j <= size(path,1) && norm(path(j,:) - ds(end,:)) > 1.5
                j = j + 1;
            end
            if j > size(path,1)
                i = size(path,1)+1;
            else
                i = j+1;
            end
            added = true;
            break;
        end
    end
    if ~added
        fp = [fp; path(i,:)]; %#ok<AGROW>
        i = i + 1;
    end
end
final_path = unique_rows(fp);
end

%% ---------------- helpers for Avoid_rectangular ----------------
function [rmin,rmax,cmin,cmax] = rect_init_expand(M, r, c)
% inicia retangulo em um unico nó e expande enquanto houver obstaculos adjacentes
rmin = r; rmax = r; cmin = c; cmax = c;
changed = true;
while changed
    changed = false;
    % expand up
    if rmin > 1 && any(M(rmin-1,cmin:cmax)==0)
        rmin = rmin - 1; changed = true;
    end
    % expand down
    if rmax < size(M,1) && any(M(rmax+1,cmin:cmax)==0)
        rmax = rmax + 1; changed = true;
    end
    % expand left
    if cmin > 1 && any(M(rmin:rmax,cmin-1)==0)
        cmin = cmin - 1; changed = true;
    end
    % expand right
    if cmax < size(M,2) && any(M(rmin:rmax,cmax+1)==0)
        cmax = cmax + 1; changed = true;
    end
end
end

function idx = find_in_path_region(path, rmin, rmax, cmin, cmax, mode)
idx = [];
for i=1:size(path,1)
    if path(i,1) >= rmin && path(i,1) <= rmax && path(i,2) >= cmin && path(i,2) <= cmax
        if strcmp(mode,'first')
            idx = i; return;
        else
            idx = i; % continue to find last
        end
    end
end
end

function [rmin2,rmax2,cmin2,cmax2, expanded] = rect_expand_one(M, rmin, rmax, cmin, cmax)
expanded = false;
rmin2=rmin; rmax2=rmax; cmin2=cmin; cmax2=cmax;
if rmin>1
    if any(M(rmin-1,cmin:cmax)==0)
        rmin2 = rmin-1; expanded = true;
    end
end
if rmax < size(M,1)
    if any(M(rmax+1,cmin:cmax)==0)
        rmax2 = rmax+1; expanded = true;
    end
end
if cmin>1
    if any(M(rmin:rmax,cmin-1)==0)
        cmin2 = cmin-1; expanded = true;
    end
end
if cmax < size(M,2)
    if any(M(rmin:rmax,cmax+1)==0)
        cmax2 = cmax+1; expanded = true;
    end
end
end

function p = nearest_free(M, pos)
[r,c] = find(M==1);
if isempty(r)
    p = pos; return;
end
dist = sqrt((r - pos(1)).^2 + (c - pos(2)).^2);
[~, idx] = min(dist);
p = [r(idx), c(idx)];
end

function path = bfs_grid_with_mask(M, start_pos, goal_pos)
% BFS clássica (4-vizinhos) retornando caminho se existir
R = size(M,1); C = size(M,2);
visited = false(R,C);
parent = zeros(R,C,2);
q = zeros(R*C,2);
head=1; tail=1;
q(tail,:) = start_pos; visited(start_pos(1),start_pos(2)) = true;
dirs = [1 0; -1 0; 0 1; 0 -1];
found=false;
while head <= tail
    cur = q(head,:); head = head + 1;
    if isequal(cur, goal_pos)
        found = true; break;
    end
    for d=1:4
        np = cur + dirs(d,:);
        if np(1)>=1 && np(1)<=R && np(2)>=1 && np(2)<=C
            if ~visited(np(1),np(2)) && M(np(1),np(2)) ~= 0
                visited(np(1),np(2)) = true;
                parent(np(1),np(2),:) = cur;
                tail = tail + 1;
                q(tail,:) = np;
            end
        end
    end
end
if ~found, path = []; return; end
% reconstruir caminho
path = goal_pos;
cur = goal_pos;
while ~isequal(cur, start_pos)
    pr = squeeze(parent(cur(1),cur(2),:))';
    if all(pr==0), break; end
    cur = pr;
    path = [cur; path];
end
end

function U = unique_rows(A)
% retorna matriz com linhas únicas na ordem de aparição
[~, ia, ~] = unique(A, 'rows', 'stable');
U = A(sort(ia), :);
end

