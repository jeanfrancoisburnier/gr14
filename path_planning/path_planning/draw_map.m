%draw_map
m = map()
m.drawMap

% x_size = 2
% x = linspace(-1,1,sqrt(N)+1)
% y = linspace(-1,1,sqrt(N)+1)
% 
% % Horizontal grid 
% for k = 1:length(y)
%   line([x(1) x(end)], [y(k) y(k)])
% end
% 
% % Vertical grid
% for k = 1:length(y)
%   line([x(k) x(k)], [y(1) y(end)])
% end

axis('equal')