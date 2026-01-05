function [start_pos, goal_pos] = find_map_start_goal(M)
% retorna [row,col] do start (valor 2) e goal (valor 3)
[r1,c1] = find(M==2);
[r2,c2] = find(M==3);
if isempty(r1) || isempty(r2)
    error('Start ou Goal nao encontrados no mapa.');
end
start_pos = [r1(1), c1(1)];
goal_pos  = [r2(1), c2(1)];
end

