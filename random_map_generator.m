clc; clear all; close all;
% choose grid size
xmax = 120;
ymax = 120;
% egocentric map stuff
new_matrix = 0.5*ones(xmax,ymax);
% ground truth map stuff
new_matrix2 = zeros(xmax,ymax);
new_matrix2(1,:)=1;new_matrix2(end,:)=1;new_matrix2(:,1)=1;
new_matrix2(:,end)=1;
% egocentric map
occ_ego_map = [0.5*ones(xmax,2) new_matrix 0.5*ones(xmax,2)]; % add two columns on each side
occ_ego_map = [0.5*ones(2,ymax+4); occ_ego_map; 0.5*ones(2,ymax+4)]; % add two rows on top and bottom
% ground truth map
occ_grt_map = [zeros(xmax,2) new_matrix2 zeros(xmax,2)];
occ_grt_map = [zeros(2,ymax+4); occ_grt_map; zeros(2,ymax+4)];
% load occ_grt_map_small_rho_60/occ_grt_map_testXY36S_rho_60.mat
% get rid of obstacles outside of map
occ_grt_map(:,1:2)=0;
occ_grt_map(:,end-1:end)=0;
occ_grt_map(1:2,:)=0;
occ_grt_map(end-1:end,:) = 0;
%save('occ_grt_map_small_rho_45X2/map100by100/occ_grt_map_testXY34_100_rho_45X2.mat','occ_grt_map')
% {0,0.5,1,2,3,5,6} --> the colors below correspond to these values in the
% order that they appear
colors = [1 1 1; 0.5 0.5 0.5;...
    0 0 0;0 0.749 1;...
    0.0941 0.5156 0.9111;1 0.1882 0.1882;...
    0 0 0.5020];
%-----------------------RANDOM OBSTACLES--------------------------
% To use Random Objects, uncomment this section
k = 0;
density_obstacles = 0;
density_threshold = 20;
% example of manual obstacles
% occ_grt_map(8,8:end-8)=1; % up % occ_grt_map(8:end-30,8)=1; % lt % occ_grt_map(8:end-30,end-8)=1; %rt % occ_grt_map(end-30,8:end-8) =1; %dn
while (density_obstacles < density_threshold)
    length = max(1,randi(5));
    x = randi(xmax-2*length)+length;
    if x == 1 || x == 2 || x == 3
        x = x+3;
    elseif x == 27 || x == 28 || x== 29 || x==30
        x = x-3;
    else
    end
    y = randi(ymax-2*length)+length;
    if y == 1 || y == 2 || y == 3
        y = y+3;
    elseif y == 28 || y == 29 || y == 30 || y == 31
        y = y-3;
    else
    end
    direction = randi(4);
    if (direction == 1)
        x = x:x+length;
    elseif (direction == 2)
        x = x-length:x;
    elseif (direction == 3)
        y = y:y+length;
    elseif (direction == 4)
        y = y-length:y;
    end
    if (sum(occ_grt_map(x,y))==1) % checking to see if the cell is already occupied (0 means obstacle present)
        continue
    end
    occ_grt_map(x,y) = 1; % changed from 1 to 0 bc of inverted logical map
    k = k + 1;
    occ_grt_map(4,4:end-3)=0; % up
    occ_grt_map(4:end-3,4)=0; % lt
    occ_grt_map(4:end-3,end-3)=0; %rt
    occ_grt_map(end-3,4:end-3) =0; %dn
    find_ones = size(find(occ_grt_map == 1),1);
    density_obstacles = (find_ones/(xmax*ymax))*100;
end
%----------------------------------------------------------- ---
% get rid of obstacles outside of map
occ_grt_map(:,1:2)=0;
occ_grt_map(:,end-1:end)=0;
occ_grt_map(1:2,:)=0;
occ_grt_map(end-1:end,:) = 0;
find_ones = size(find(occ_grt_map == 1),1);
density_obstacles = (find_ones/(xmax*ymax))*100
%---------------------------------------------------------------
grtFigure = figure('Name','ground truth map');
position_figure(2,3,1)
%heatmap(occ_grt_map)
%surf(flip(occ_grt_map,1)); view(2);
imagesc(occ_grt_map);
colormap(colors);
%save('RGB_filter_demo_maps/map40by40/occ_grt_map_testXY3_40_rho_20X2.mat','occ_grt_map')
%load occ_grt_map_small_rho_20X2/map20by20/occ_grt_map_testXY1_20_rho_20X2.mat 
\end{lstlisting}
