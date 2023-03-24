function a_star_plot(map, costs, route,start, goal)
% A_STAR_PLOT plots the output of A_STAR.
%   MAP is the MAP input to A_STAR
%   COSTS is the COST input to A_STAR
%   ROUTE is the output of A_STAR

% Alex Ranaldi
% alexranaldi@gmail.com

if ~islogical(map)
    error('map must be logical')
end
mapSize = size(map);
hf = figure('Name','Global Planner Optimal Path to Goal','Color','white');
position_figure(2,3,4)
ha = axes( ...
    'parent', hf, ...
    'ydir','reverse',...
    'xdir','normal',...
    'xlim', [1 mapSize(1)+2], ...
    'ylim', [1 mapSize(2)+2], ...
    'xtick', 1:mapSize(1)+2, ...
    'ytick', 1:mapSize(2)+2 ...
);
grid on;axis tight 
% normalize costs
minCost = min(costs(:));
maxCost = max(costs(:));
mmCost = maxCost - minCost;
if mmCost == 0
    mmCost = 1;
end
costs = (costs(:) - minCost) / mmCost;
colorIndex = fix(costs*100)+1;
colors = jet(101); % colors map to distinguish between costs 
%colors = [0 0 0; 1 0 0];
for k = 1 : numel(map)
    [r, c] = ind2sub(mapSize, k);
    if map(k)
        patch( ...
            'parent', ha, ...
            'xdata', [c c c+1 c+1], ...
            'ydata', [r r+1 r+1 r], ...
            'facecolor', colors(colorIndex(k),:) ...
            );
    end
end
[r, c] = ind2sub(mapSize, route);
for k = 1 : length(route)
    line('parent', ha, ...
        'xdata', c + 0.5, ...
        'ydata', r + 0.5, ...
        'linewidth', 2, ...
        'color', 'r', ...
        'linestyle', '-.' ...
        );
end
hold on ;
% note the algorithm has shifted the "start" and "goal" coordinates by 0.5
% such that the path is through the cells and not the nodes of the cell.
% notice [y,x] instead of [x,y] bc of ind2sub()
[start_y,start_x]=ind2sub(mapSize,start);
[goal_y,goal_x]=ind2sub(mapSize,goal);
% displaying the start and goal on plot
text = sprintf("START:: X: %4.2f Y: %4.2f; GOAL:: X: %4.2f Y: %4.2f",...
    start_x+0.5,start_y+0.5, goal_x+0.5,goal_y+0.5);
title(text)
plot(start_x + 0.5,start_y + 0.5,'s','MarkerFaceColor','c','MarkerSize',12)
plot(goal_x + 0.5,goal_y + 0.5,'p','MarkerFaceColor','y','MarkerSize',12)
xticks([0 10 20 30 40 50 60])
yticks([0 10 20 30 40 50 60])
\end{lstlisting}
