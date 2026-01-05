function idx = select_reference_map(maps)
% seleciona mapa com maior número de obstáculos (mais denso)
n = length(maps);
counts = zeros(1,n);
for i=1:n
    counts(i) = sum(maps{i}(:) == 0);
end
[~, idx] = max(counts);
end

