function nbs = get_free_neighbors(map, pos)
% Retorna todos os vizinhos livres (8 direções) da célula 'pos'
% 0 = obstáculo, 1 = livre, 2/3 = start/goal (também livres)
[R,C] = size(map);
i = pos(1);
j = pos(2);
dirs = [ 1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1 ];
nbs = [];

for d = 1:size(dirs,1)
    ni = i + dirs(d,1);
    nj = j + dirs(d,2);
    if ni >= 1 && ni <= R && nj >= 1 && nj <= C
        if map(ni,nj) ~= 0
            nbs = [nbs; ni nj]; %#ok<AGROW>
        end
    end
end
end

