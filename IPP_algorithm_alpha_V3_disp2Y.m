%save('save_occ_ego_map.mat','occ_ego_map')
%clc; clear all; close all;
%INITIALIZING STUFF
% grid size
xmax = 40;
ymax = 40;
% egocentric map stuff
new_matrix = 0.5*ones(xmax,ymax);
% ground truth map stuff
new_matrix2 = zeros(xmax,ymax);
new_matrix2(1,:)=1;new_matrix2(end,:)=1;new_matrix2(:,1)=1;
new_matrix2(:,end)=1;
% new_matrix2(14,:)=1; % add dividing line in
%occ_grt_map
% egocentric map
occ_ego_map = [0.5*ones(xmax,2) new_matrix 0.5*ones(xmax,2)]; % add two columns on each side
occ_ego_map = [0.5*ones(2,ymax+4); occ_ego_map; 0.5*ones(2,ymax+4)]; % add two rows on top and bottom
% ground truth map
occ_grt_map = [zeros(xmax,2) new_matrix2 zeros(xmax,2)];
occ_grt_map = [zeros(2,ymax+4); occ_grt_map; zeros(2,ymax+4)];
% loads the occupancy ground truth map stored (occ_grt_map)
%load occ_grt_map_small_rho_60/occ_grt_map_testXY39S_rho_60.mat 
%load occ_grt_map_small_rho_45X2/map80by80/occ_grt_map_testXY21_80_rho_45X2.mat 
load RGB_filter_demo_maps/map40by40/occ_grt_map_testXY1_40_rho_20X2.mat
%load folderName/fileName;
% make free space
% occ_grt_map(8:12,4)=0; % occ_grt_map(14,7)=0;
% add obstacles 
% occ_grt_map(18:28,7)=1; % % occ_grt_map(3:10,12)=1;
%color map
% {0,0.5,1,2,3,5,6} --> the colors below correspond to these values in the
% order that they appear
colors = [1 1 1; 0.5 0.5 0.5;...
    0 0 0;0 0.749 1;...
    0.0941 0.5156 0.9111;1 0.1882 0.1882;...
    0 0 0.5020];
grtFigure = figure('Name','ground truth map');
position_figure(2,3,1)
imagesc(occ_grt_map);
colormap(colors);
% expansion map
occ_exp_map = occ_ego_map;
% initial position of the robot
n = 3;
switch n
    case 1 % top left
        initPose = [8 8];
    case 2 % top right
        initPose = [4 28];
    case 3 % bottom left
        initPose = [25 4];
    case 4 % bottom right
        initPose = [24 20];
    case 5 % middle
        initPose = [16 16];
    otherwise % start the robot at a random position in the map
        initPose = [randi([4,27],1,1) randi([4,28],1,1)];
end
nowPose = initPose;
% number of iterations for the simulation (local planner)
N1 = 500; % make it large so when grt_maps get larger there is no need to change #; calls each instance of the local planner 'N' times
N2 = 1; % this counter will be used for the second instance of the local planner
% NOTE: the reason we only want to show the robot w/ its sensor model in the begining of the simulation is so that the occ_exp_map does not become overwritten...
% initial robot position
occ_exp_map(initPose(1),initPose(2))=5;
% we need to construct the sensor model at our initial robot position
occ_exp_map(initPose(1)-1,initPose(2)-1:initPose(2)+1)=3;
occ_exp_map(initPose(1)+1,initPose(2)-1:initPose(2)+1)=3;
occ_exp_map(initPose(1),initPose(2)-1)=3;
occ_exp_map(initPose(1),initPose(2)+1)=3;
% construct the sensor expansion model
occ_exp_map(initPose(1)-2,initPose(2)-2:initPose(2)+2)=6;
occ_exp_map(initPose(1)+2,initPose(2)-2:initPose(2)+2)=6;
occ_exp_map(initPose(1)-1:initPose(1)+1,initPose(2)-2)=6;
occ_exp_map(initPose(1)-1:initPose(1)+1,initPose(2)+2)=6;
% flags to keep track of local and global planner being active
GP_active = 0;
LP_active = 1; % by defualt the local planner will be active
total_robot_steps = 0;
GP_robot_steps = 0;
LP_robot_steps = 0;
tic_entire_loop=tic; % timing loop completion
final_route = 1; % just to start the while loop.
while final_route ~= 0 % could change to condition that depends on exploration of map being 100 %
    [occ_ego_map,return_pose,poses,LP_active,GP_active,egoFigure,explore_all_flag,LP3A_robot_steps] = ...
        local_planner_IPP_V3A_disp2Y(nowPose,N1,occ_grt_map, occ_ego_map,...
        occ_exp_map,colors,xmax,ymax,total_robot_steps); % added total_robot_steps as input
    % incrementing robot steps
    total_robot_steps = total_robot_steps + LP3A_robot_steps;
    LP_robot_steps = LP_robot_steps + LP3A_robot_steps;
    if explore_all_flag==1
        % break out of main for loop once the map has been fully explored
        disp('IPP_algo:: break from while loop.')
        toc_entire_loop = toc(tic_entire_loop);
        fprintf('Explored %4.2f perc. obstacle dense map in %4.2f seconds with total of %4.2f robot moves.\n',density_obstacles,toc_entire_loop,total_robot_steps)
        % open the ground truth figure again...
        grtFigure = figure('Name','ground truth map');
        position_figure(2,3,1)
        % commented on 08/04/22
        imagesc(occ_grt_map);
        colormap(colors);
        %placed here on 08/04/22
        egoFigure = figure('Name','egocentric map');
        position_figure(2,3,2)
        imagesc(occ_ego_map);
        colormap(colors);
        break;
    end
    while (GP_active == 1 && LP_active == 0) 
        disp("GP_active=1 && LP_active=0")
        % deactivating global planner for next iteration GP_active = 0;
        %disp("start pose from local planner to global planner")
        start_pose = return_pose; %[Y, X]
        % call global planner with the robot posistion from the local planner
        final_route = global_planner_IPP_V3_disp2Y(occ_ego_map, start_pose);
        final_route = flip(final_route); % have the path be from start to finish
        mapSize = size(occ_ego_map);
        % added on 08/02/22:: this code will only be executed when the robot has a goal that is not accesible because it is enclosed by obstacles. The code below is based on an RGB filtering. The value of RGB where the robot is located and the known obstalces, based on the occ_ego_map, will not be changed, but the other cells willchange to occuppied because they are not accessible. 
        if isempty(final_route) == 1
            % call this function
            binaryImage = occ_ego_map -1;
            % Label the blobs
            [labeledImage, numBlobs] = bwlabel(binaryImage);
            % Let's assign each blob a different color to visually show the user the distinct blobs.
            coloredLabels = label2rgb (labeledImage, 'parula(5)', 'k', 'noshuffle'); % pseudo random color labels
            figure(33);
            imagesc(coloredLabels);
            RGB_occ_ego_map = double(coloredLabels(:,:,3));
            robot_RGB = RGB_occ_ego_map(start_pose(1)-0.5,start_pose(2)-0.5);
            % compare the occ_ego_map to the RGB_occ_ego_map 
            % find the indeces of the cells that do not equal robot_RGB value or 0. Then, change the values in the occ_ego_map to 1, so that the robot "knows" that those cells are not accesible.
            [row5,col5 ,~]=find(RGB_occ_ego_map==0);
            [row6,col6 ,~]=find(RGB_occ_ego_map==robot_RGB);
            % append rows and cols
            row7 = [row5; row6]; col7 = [col5; col6];
            for row_i4 = 1:mapSize(1)
                for col_i4 = 1:mapSize(2)
                    if RGB_occ_ego_map(row_i4,col_i4) == 0 || RGB_occ_ego_map(row_i4,col_i4) == robot_RGB
                    else
                        % change all cellls that are not accessible to the robot to be occupied...
                        occ_ego_map(row_i4,col_i4) = 1;
                    end
                end
            end
            % call the global planner again. This time with the updated occ_ego_map. 
            start_pose,egoFigure);
            final_route = global_planner_IPP_V3_disp2Y(occ_ego_map, start_pose);
            final_route = flip(final_route); % have the path be from start to finish
        end
        if final_route == 0
            disp('IPP_algo:: BREAK FROM THE LOOP. All accesible areas of the map have been explored.')
            toc_entire_loop = toc(tic_entire_loop);
            fprintf('Explored %4.2f perc. obstacle dense map in %4.2f seconds with total of %4.2f robot moves.\n',density_obstacles,toc_entire_loop,total_robot_steps)
            break;
        end
        [r, c] = ind2sub(mapSize, final_route);
    [occ_ego_map,return_pose,LP_active,GP_active,egoFigure,explore_all_flag,LP3B_robot_steps] = ...
        local_planner_IPP_V3B_disp2Y(final_route,occ_grt_map, occ_ego_map,...
            occ_exp_map,colors,xmax,ymax,total_robot_steps);
        % incrementing robot steps
        total_robot_steps = total_robot_steps + LP3B_robot_steps
        LP_robot_steps = LP_robot_steps + LP3B_robot_steps;
    end
    % this pose will now be given to the local planner
    nowPose = return_pose; %nowPose = [r(end),c(end)];
    % (DONE) have the robot travel on the global planner path until it reaches the goal
    % @ if the robot travels through an unexplored region, break out of the path to the goal, and start the local planner at that specific location (keep a threshold for directional sums)
    % @ if the robot encounters not unexplored regions, then have the robot continue on the path to the goal
    % @ if an obstacle was detected along the path to the goal, then break then stop the robot from continuing on the path, and run generate a new path to the goal
    close all; % close all figures 
    if final_route == 0
        % open the ground truth figure again...
        grtFigure = figure('Name','ground truth map');
        position_figure(2,3,1)
        imagesc(occ_grt_map);
        colormap(colors);
        egoFigure = figure('Name','egocentric map');
        position_figure(2,3,2)
        imagesc(occ_ego_map);
        colormap(colors);
    end
end
