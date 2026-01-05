function draw_map_with_path(M, path)
  % M: mapa, path: Nx2 coordenadas (linha, coluna)
  img = M;
  im = zeros(size(M,1), size(M,2), 3);
  im(:,:,1) = (M ~= 0); % vermelho para áreas livres
  im(:,:,2) = (M == 1);
  imagesc(im); axis equal tight; hold on;
  if ~isempty(path)
    plot(path(:,2), path(:,1), '-r', 'LineWidth', 2);
    plot(path(1,2), path(1,1), 'ob', 'MarkerFaceColor', 'b');
    plot(path(end,2), path(end,1), 'ok', 'MarkerFaceColor', 'k');
  end
  hold off;
end

