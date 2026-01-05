function maps = generate_example_maps()
% Gera 3 mapas 50x50 conforme Tabela 1 do artigo
rng(1); % reproduzibilidade
dims = [50,50];
maps = cell(1,3);
% probabilidades / pesos do artigo (Table 1)
wj = [0.1931, 0.2964, 0.5105];
obst_weights = [0.3466, 0.2881, 0.3653];

for k = 1:3
    p = obst_weights(k);        % densidade de obstáculo
    M = ones(dims);             % 1 = livre
    mask = rand(dims) < p;
    M(mask) = 0;                % 0 = obstáculo
    if k==3
        % conforme artigo: mapa 3 tem start=(6,42) e goal=(41,7)
        M(6,42) = 2; M(41,7) = 3;
    else
        % escolher start/goal aleatórios em células livres
        frees = find(M==1);
        s = frees(randi(length(frees)));
        g = frees(randi(length(frees)));
        M(s) = 2; M(g) = 3;
    end
    maps{k} = M;
end
end

