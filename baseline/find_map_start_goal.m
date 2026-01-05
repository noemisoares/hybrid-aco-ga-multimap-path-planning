function [start_pos, goal_pos] = find_map_start_goal(M)
% Localiza as coordenadas do ponto inicial (2) e final (3)
[r1, c1] = find(M == 2);
[r2, c2] = find(M == 3);
if isempty(r1) || isempty(r2)
    error('Start ou Goal não encontrados no mapa.');
end
start_pos = [r1(1), c1(1)];
goal_pos = [r2(1), c2(1)];
end

