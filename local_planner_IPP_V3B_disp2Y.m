function [occ_ego_map, return_pose,LP_active,GP_active, egoFigure,explore_all_flag,LP3B_robot_steps] = local_planner_IPP_V3B_disp2Y(route,occ_grt_map, occ_ego_map,occ_exp_map,colors,xmax,ymax,total_robot_steps)
% grtFigure = figure('Name','ground truth map');
% position_figure(2,3,1)
% %heatmap(occ_grt_map)
% surf(flip(occ_grt_map,1)); view(2);
% colormap(colors);
egoFigure = figure('Name','egocentric map');
position_figure(2,3,2)
%heatmap(occ_ego_map)
surf(flip(occ_ego_map,1)); view(2);
colormap(colors);
expFigure = figure('Name','expansion map');
position_figure(2,3,3)
%heatmap(occ_exp_map)
surf(flip(occ_exp_map,1)); view(2);
colormap(colors);
% exp2Figure = figure('Name','expansion model update 7x7'); figure(exp2Figure);
% position_figure(2,3,4)
% %heatmap(mat_sm7_exp)
exp3Figure = figure('Name','expansion model update 5x5'); figure(exp3Figure);
position_figure(2,3,5)
%heatmap(mat_sm5_exp)
offset_0p5 = 0.5;
poseFigure = figure('Name','All time poses of Robot'); figure(poseFigure);
position_figure(2,3,6)
% plot(poses(1,:)+offset_0p5,poses(1,:)+offset_0p5,'ob')
xlim([0 xmax+2+3]) % +2 (border), +3 (equal spacing left and right)
ylim([0 ymax+2+3])
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
hold on;

v = [3 xmax+3;3 3;ymax+3 3;ymax+3 xmax+3];f = [1 2 3 4];
patch('Faces',f,'Vertices',v,...
'EdgeColor','black','FaceColor','none','LineWidth',3);
% timing stuff
% t1 = zeros(1,N);t2=zeros(1,N);t3=zeros(1,N);t4=zeros(1,N);t5=zeros(1,N);
% toc_loop=zeros(1,N);
% sum_up = zeros(1,N);sum_dn=zeros(1,N);sum_lt=zeros(1,N);sum_rt=zeros(1,N);

% tic_entire_loop=tic; % timing loop completion
mapSize = size(occ_ego_map); [r,c] = ind2sub(mapSize,route)
% repeat the for loop until the robot explores the goal break the for loop if there are unexplored regions to the robots end goal; explore the regions 
N2 = length(r);
explore_all_flag=0;
% extracting the end goal from nowPose()
potential_goal_X1 = c(end) potential_goal_Y1 = r(end)
count_steps2 = total_robot_steps;
for i2=1:N2
% call the local planner with this current robot location "nowPose"
nowPose=[r(i2),c(i2)];
poses(:,i2)=nowPose; % for plotting poses
%     tic_loop(i)=tic;
fprintf("Robot Location:: X:%4.2f, Y:%4.2f @ iter. %d \n",nowPose(2)+0.5,nowPose(1)+0.5,i2)
%     t1_tic(i) = tic;
%check to see if the  sensor model occuppies an occupied element in the occupancy matrix
[occ_ego_map, obs_flag_vector]=check_occ_sensor_model(nowPose,occ_grt_map,occ_ego_map);
%     t1(i)= toc(t1_tic(i));
%     t2_tic(i)=tic;
perc_exp = percent_explored(occ_grt_map,occ_ego_map);
%     t2(i)=toc(t2_tic(i));
%     pause(0.01);
figure(egoFigure);
%heatmap(occ_ego_map)
%surf(flip(occ_ego_map,1)); view(2);
imagesc(occ_ego_map);
colormap(colors);
text = sprintf("Robot Location:: X: %4.2f Y: %4.2f @ iteration %d. %% EXP::%4.2f%%",nowPose(2)+0.5,nowPose(1)+0.5,i2,perc_exp);
title(text)
% assign the current occ_ego_map to the occ_exp_map
occ_exp_map = occ_ego_map;
%     pause(0.01);
figure(expFigure);
%heatmap(occ_exp_map)
%surf(flip(occ_exp_map,1)); view(2);
imagesc(occ_exp_map);
colormap(colors);
% returns occ_exp_map and the 5x5 representation of sensor model probably no need to return 5x5 sensor model
%     t3_tic(i)=tic;
[occ_exp_map,mat_sm5_exp]=check_occ_exp_sensor_model(nowPose,occ_exp_map,exp3Figure,colors);
%     t3(i)=toc(t3_tic(i));
% obtain next direction of travel
%     t4_tic(i)=tic;
[nowPose,vect_sum,vect_sum2,moved_all_flags]=directional_sums(nowPose,occ_exp_map, obs_flag_vector, i2);
% directional sums
sum_up(i2)=vect_sum(1); sum_dn(i2)=vect_sum(2);
sum_lt(i2)=vect_sum(3);sum_rt(i2)=vect_sum(4);
% flags indicating movement
moved_up_flag =moved_all_flags(1); moved_dn_flag =moved_all_flags(2);
moved_lt_flag =moved_all_flags(3); moved_rt_flag =moved_all_flags(4);
%     t4(i)=toc(t4_tic(i));
%     t5_tic(i)=tic;
%     t5(i)=toc(t5_tic(i));
% note vect_sum contains [sum_up sum_dn sum_lt sum_rt]
%     poses(:,i+1)=nowPose; % for plotting poses
%     pause(0.01);
figure(expFigure);
%heatmap(occ_exp_map)
%surf(flip(occ_exp_map,1)); view(2);
imagesc(occ_exp_map);
colormap(colors);
text = sprintf("Directional Sums:: up: %d; dn: %d; lt: %d; rt: %d @ iteration %d", ...
    vect_sum(1),vect_sum(2),vect_sum(3),vect_sum(4),i2);
title(text)
% plot poses of the robot
%filled_poses = nonzeros(poses);
figure(poseFigure);
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
xlim([0 xmax+2+3]) % +2 (border), +3 (equal spacing left and right)
ylim([0 ymax+2+3])
hold on;
plot(poses(2,i2)+offset_0p5,poses(1,i2)+offset_0p5,'ob')
text = sprintf("Robot Location:: X: %4.2f Y: %4.2f @ iteration %d", poses(2,i2)+offset_0p5,poses(1,i2)+offset_0p5,i2);
title(text)
% visualize the robot location 
figure(egoFigure);
hold on;
%plot(nowPose(2)+offset_0p5,nowPose(1)+offset_0p5,'ro')
plot(nowPose(2),nowPose(1),'rx')
%------------------------06/03/22-------------------------------
%pose_direc_info(i,:) = [x, y, sum_up, sum_dn, sum_lt, sum_rt, iteration]
disp("LP_V3B:: Pose and Directional Sum Info")
pose_direc_info(i2,:) = [poses(2,i2)+0.5, poses(1,i2)+0.5, ...
    vect_sum(1), vect_sum(2), vect_sum(3), vect_sum(4), i2];
%---------------------------------------------------------------
%nonzero robot poses
LP3B_robot_steps =size(nonzeros(poses(1,:)),1);
%pause()
%     toc_loop(i)=toc(tic_loop(i));
% CRITERION FOR STOPPING SIMULATION
% stopping the program once the map has been explored
if perc_exp==100
    disp('Congrats, the map has been fully explored.')
    % everything in the map has been explored
    explore_all_flag=1;
    % nonzero robot poses
    LP3B_robot_steps =size(nonzeros(poses(1,:)),1);
    break;
end
hori_neighbors_flag = 0; vert_neighbors_flag = 0;
if i2<N2
    nextPose=[r(i2+1),c(i2+1)];
    % goal is located between two neighboring obstacles, horizontally
    if (occ_ego_map(nextPose(1),nextPose(2)-1) == 1 && occ_ego_map(nextPose(1),nextPose(2)+1) == 1) && occ_ego_map(nextPose(1),nextPose(2)) == 1
        hori_neighbors_flag = 1
    end
    % goal is lcoated between two neighboring obstacles, vertically 
    if  (occ_ego_map(nextPose(1)-1,nextPose(2)) == 1 && occ_ego_map(nextPose(1)+1,nextPose(2)) == 1) && occ_ego_map(nextPose(1),nextPose(2)) == 1
        vert_neighbors_flag = 1
    end
    % stop the local planner if the cell is occupied or unknown
    % note: 0.5: unknown status; 1: occuppied status
    test_values6 = [0.5 1];
    occ_unk_cell_flag = 0;
    if sum(ismember(occ_ego_map(nextPose(1),nextPose(2)),test_values6)) ==1
        occ_unk_cell_flag = 1
    end
    if  occ_unk_cell_flag== 1
        return_pose = [r(i2),c(i2)];
        % nonzero robot poses
        LP3B_robot_steps =size(nonzeros(poses(1,:)),1);
        break;
elseif (hori_neighbors_flag == 1 || vert_neighbors_flag == 1)
        return_pose = [r(i2),c(i2)];
          % nonzero robot poses
           LP3B_robot_steps =size(nonzeros(poses(1,:)),1);
        disp('full send it.')
        break;
    else
        % run_globalPlanner_flag = 0;
    end
end
% Update: 07/27/22
% add the code here where the robot will stop traveling on a path to a chosen goal if there are unexplored regions closer to the robot while the robot is on its path to the first chosen goal.
% check it every 5-10 iterations responds to avoid computation costs    
%- compare the distance to the chosen first goal to other potential nearby goals.
if mod(i2,9) == 0 % check every 9 iterations of the code 
    current_pos_X1 = c(i2);
    current_pos_Y1 = r(i2);
    %mapSize = size(occ_ego_map); % redundant
    x_size1 = mapSize(1); y_size1 = mapSize(2); % redundant
    count = 0;
    % 05/29/22
    % note the +/- 2 is added/subtracted to the indexing variables i and j such that the unexplored regions beyond the walls are not potential goals...
    for i = 1+2:x_size1-2 % iterate through rows
        for j = 1+2:y_size1-2 % iterate through cols
            if occ_ego_map(i,j)==0.5
                count = count+1;
                %disp(count)
                % unconventional... for (i,j), but it works
                potential_goals_XX(count) = j; % X is for COLS
                potential_goals_YY(count) = i; % Y is for ROWS
            end
        end
    end
    % start_pose = [y,x]
    robot_location = [potential_goal_X1-0.5, potential_goal_Y1-0.5]; % nowPose
    potential_goals = [robot_location(1) robot_location(2);potential_goals_XX' potential_goals_YY'];
    D = pdist(potential_goals);
    Z = squareform(D);
    % keep track of coordinates and their respective distnace from the start
    % robot location... probably use cells a structure or cell array
    dist_potential_goals = Z(1,:);
    dist_potential_goals_2 = [potential_goals dist_potential_goals'];
    % get rid of the robot location (first row)
    dist_potential_goals_2 = dist_potential_goals_2(2:end,:);
    search_min = min(dist_potential_goals_2(:,3));
    search_min_idx = find(dist_potential_goals_2(:,3)==search_min);
    size_search_min_idx = size(search_min_idx,1);
    if size_search_min_idx == 1
        % extracting x and y coordinates of min distance found
        disp("Search_min_idx = 1")
        robot_goal_x = dist_potential_goals_2(search_min_idx,1);
        robot_goal_y = dist_potential_goals_2(search_min_idx,2);
    elseif size_search_min_idx == 0
        disp("This is definently an error.")
    else
        disp('Search_min_idx > 1')
        % select the first coordinate as a goal; ignore the rest since they are
        % the equidistant
        robot_goal_x = dist_potential_goals_2(search_min_idx(1),1);
        robot_goal_y = dist_potential_goals_2(search_min_idx(1),2);
    end
    D1 = sqrt((potential_goal_Y1-current_pos_Y1)^2 + (potential_goal_X1-current_pos_X1)^2);
    D2 = sqrt((robot_goal_y-current_pos_Y1)^2 + (robot_goal_x-current_pos_X1)^2);
    if D2<D1
        disp('Original goal terminated... chosing new goal since it is closer to the robot.')
        % nonzero robot poses
        LP3B_robot_steps =size(nonzeros(poses(1,:)),1);
        break;
    end
end
% return_pose will return the end goal location to the main script only if the robot stays on the path to the end goal. 
return_pose = [r(i2),c(i2)]; % for testing purposes
% turning both the GP and LC
LP_active=0; GP_active=0;
end
% num = size(nonzerso(toc_loop));
% avg_loop_time =sum(nonzeros(toc_loop))/num;
% toc_entire_loop = toc(tic_entire_loop);
% function will check if the sensor model in the occ_exp_map occupies an occupied cell in the occ_grt_map it will return which cells are accopied so the robot does not travel in that direction
function [occ_ego_map, obs_flag_vector]=check_occ_sensor_model(nowPose,occ_grt_map,occ_ego_map)
% to make it easy, why not extract the values of the group of cells in the cc_grt_map that the sensor model occupies in the occ_exp_map, then check which cells are occupied (value of 1) and id them. Transfer the gained map information to the occ_ego_map.
    mat_sm3_grt = zeros(3); % matrix extracted from occ_grt_map
    mat_sm3_ego = zeros(3); % matrix extracted from occ_ego_map
    for row=1:3
        if row==1
            mat_sm3_grt(row,:)= occ_grt_map(nowPose(1)-1,nowPose(2)-1:nowPose(2)+1);
            mat_sm3_ego(row,:)= occ_ego_map(nowPose(1)-1,nowPose(2)-1:nowPose(2)+1);
        elseif row==2
            mat_sm3_grt(row,:)= occ_grt_map(nowPose(1),nowPose(2)-1:nowPose(2)+1);
            mat_sm3_ego(row,:)= occ_ego_map(nowPose(1),nowPose(2)-1:nowPose(2)+1);
        elseif row==3
            mat_sm3_grt(row,:)= occ_grt_map(nowPose(1)+1,nowPose(2)-1:nowPose(2)+1);
            mat_sm3_ego(row,:)= occ_ego_map(nowPose(1)+1,nowPose(2)-1:nowPose(2)+1);
        else
            % do nothing.
        end
    end
% compare extracted 3x3 matrix from occ_grt_map to the occ_ego_map. If a cell is occupied in the occ_grt_map then transfer that information to the occ_ego_map. This information will then be used in the occ_exp_map to aid the robot in prioritizing a direction for exploration
    for row=1:3
        for col=1:3
            if mat_sm3_grt(row,col)==1
                mat_sm3_ego(row,col)=1;
            else
                mat_sm3_ego(row,col)=0;
            end
        end
    end
    %-----------------------------------------------------------
    [row4,col4,~] = find(mat_sm3_ego==1);
    obs_rt_flag = 0; obs_lt_flag = 0; obs_dn_flag = 0; obs_up_flag = 0;
    obs_up_mid_flag = 0; obs_lt_mid_flag = 0; obs_rt_mid_flag = 0; obs_dn_mid_flag = 0;
    % only display the sensor model matrix if there exist obstacles
    if isempty(row4)==0
        disp("sensor model matrix")
        disp(mat_sm3_ego)
        % determine the value of each cell independently and then add them together.
        % top cells
        top_lt_corner = mat_sm3_ego(1,1);
        top_mid = mat_sm3_ego(1,2);
        top_rt_corner = mat_sm3_ego(1,3);
        % middle cells excluding center cell
        lt_mid = mat_sm3_ego(2,1);
        rt_mid = mat_sm3_ego(2,3);
        % bottom cells 
        bot_lt_corner = mat_sm3_ego(3,1);
        bot_mid = mat_sm3_ego(3,2);
        bot_rt_corner =  mat_sm3_ego(3,3);
        sum_obs_rt = sum([top_rt_corner rt_mid bot_rt_corner]);
        sum_obs_lt = sum([top_lt_corner lt_mid bot_lt_corner]);
        sum_obs_up = sum([top_lt_corner top_mid top_rt_corner]);
        sum_obs_dn = sum([bot_lt_corner bot_mid bot_rt_corner]);
        if sum_obs_rt == 3
            obs_rt_flag = 1;
            disp("obstacle to the RIGHT of robot")
        end
        if sum_obs_lt == 3 
            obs_lt_flag = 1;
            disp("obstacle to the LEFT of robot")
        end
        if sum_obs_dn == 3 
            obs_dn_flag = 1;
            disp("obstacle to the DOWN of robot")
        end
        if sum_obs_up == 3
            obs_up_flag = 1;
            disp("obstacle to the UP of robot")
        end
        % need to check if the middle cell in all directions is occupied (value of 1). If it is occupied then do not let the robot travel using that cell.
        if top_mid == 1 && obs_up_flag ~= 1
            obs_up_mid_flag = 1;
            disp("WARNING: obstacle located at top_mid cell")
        end
        if lt_mid == 1 && obs_lt_flag ~= 1
            obs_lt_mid_flag = 1;
            disp("WARNING: obstacle located at lt_mid cell")
        end
        if rt_mid == 1 && obs_rt_flag ~= 1
            obs_rt_mid_flag = 1;
            disp("WARNING: obstacle located at rt_mid cell")
        end
        if bot_mid == 1 && obs_dn_flag ~= 1
            obs_dn_mid_flag = 1; 
            disp("WARNING: obstacle located at bot_mid cell")
        end
        obs_flag_vector = [obs_up_flag obs_up_mid_flag;
                           obs_dn_flag obs_dn_mid_flag;
                           obs_lt_flag obs_lt_mid_flag;
                           obs_rt_flag obs_rt_mid_flag];
    end
    %-----------------------------------------------------------
    obs_flag_vector = [obs_up_flag obs_up_mid_flag;
        obs_dn_flag obs_dn_mid_flag;
        obs_lt_flag obs_lt_mid_flag;
        obs_rt_flag obs_rt_mid_flag];
    % take the mat_sm3_ego matrix generated in the previouse lines of codes and insert the matrix into the appropriate location in the occ_ego_map.
    for row=1:3
        if row==1
        occ_ego_map(nowPose(1)-1,nowPose(2)-1:nowPose(2)+1)= mat_sm3_ego(row,:);
        elseif row==2
            occ_ego_map(nowPose(1),nowPose(2)-1:nowPose(2)+1)= mat_sm3_ego(row,:);
        elseif row==3
        occ_ego_map(nowPose(1)+1,nowPose(2)-1:nowPose(2)+1)= mat_sm3_ego(row,:);
        else
            % do nothing.
        end
    end
end
% similiar stuff to function above.
function [occ_exp_map,mat_sm5_exp]=check_occ_exp_sensor_model(nowPose,occ_exp_map,exp3Figure,colors)
    mat_sm5_exp= zeros(5); % matrix extracted from occ_ego_map
    for row=1:5
        if row==1
            mat_sm5_exp(row,:)= occ_exp_map(nowPose(1)-2,nowPose(2)-2:nowPose(2)+2);
        elseif row==2
            mat_sm5_exp(row,:)= occ_exp_map(nowPose(1)-1,nowPose(2)-2:nowPose(2)+2);
        elseif row==3
            mat_sm5_exp(row,:)= occ_exp_map(nowPose(1),nowPose(2)-2:nowPose(2)+2);
        elseif row==4
            mat_sm5_exp(row,:)= occ_exp_map(nowPose(1)+1,nowPose(2)-2:nowPose(2)+2);
        elseif row==5
            mat_sm5_exp(row,:)= occ_exp_map(nowPose(1)+2,nowPose(2)-2:nowPose(2)+2);
        else
            % do nothing.
        end
    end
    % look identify the cells that have a value of 1 in mat_sm5_exp, then check the surrounding cells: change cells that are in direct contact with an occupied cell from 0.5 to 2. If not in direct contact then change from 0.5 to 6.
    [row,col,v]=find(mat_sm5_exp==1);
    for n=1:length(row)
        % do nothing.
        % consider the case when it is row 1 then we cannot subtract 1 bc of indexing issue, i.e., no row 0
        if row(n)==1
            % do nothing
        else
            %-up-
            if mat_sm5_exp(row(n)-1,col(n))== 0.5
                mat_sm5_exp(row(n)-1,col(n)) =2;
            elseif mat_sm5_exp(row(n)-1,col(n))== 1
                mat_sm5_exp(row(n)-1,col(n))= 1;
            else
                mat_sm5_exp(row(n)-1,col(n))=0;
            end
        end
        if row(n)==5
            % do nothing
        else
            %-down-
            if mat_sm5_exp(row(n)+1,col(n))== 0.5
                mat_sm5_exp(row(n)+1,col(n))=2;
            elseif mat_sm5_exp(row(n)+1,col(n))== 1
                mat_sm5_exp(row(n)+1,col(n))=1;
            else
                mat_sm5_exp(row(n)+1,col(n))=0;
            end
        end
        if col(n)==1
            % do nothing
        else
            %-left-
            if mat_sm5_exp(row(n),col(n)-1) == 0.5
                mat_sm5_exp(row(n),col(n)-1) = 2;
            elseif mat_sm5_exp(row(n),col(n)-1) == 1
                mat_sm5_exp(row(n),col(n)-1) =1;
            else
                mat_sm5_exp(row(n),col(n)-1) =0;
            end
        end
        if col(n)==5
            % do nothing
        else
            %-right-
            if mat_sm5_exp(row(n),col(n)+1) == 0.5
                mat_sm5_exp(row(n),col(n)+1) = 2;
            elseif mat_sm5_exp(row(n),col(n)+1) ==1
                mat_sm5_exp(row(n),col(n)+1) =1;
            else
                mat_sm5_exp(row(n),col(n)+1) =0;
            end
        end
    end
    % uncommented 05/27/22
    figure(exp3Figure);
%         heatmap(mat_sm5_exp,'CellLabelColor','none')
    %surf(flip(mat_sm5_exp,1)); view(2);
    imagesc(mat_sm5_exp);
    colormap(colors);
    [row2,col2,~]=find(mat_sm5_exp==0.5);
    for n=1:length(row2)
        %disp(n)
        mat_sm5_exp(row2(n),col2(n))=6;
    end
    % pause(0.01)
    figure(exp3Figure)
    % uncommented 05/27/22
%         heatmap(mat_sm5_exp,'CellLabelColor','none')
    surf(flip(mat_sm5_exp,1)); view(2);
    colormap(colors);
    % update the occ_exp_map with mat_sm5_exp
    for row=1:5
        if row==1
            occ_exp_map(nowPose(1)-2,nowPose(2)-2:nowPose(2)+2)= mat_sm5_exp(row,:);
        elseif row==2
            occ_exp_map(nowPose(1)-1,nowPose(2)-2:nowPose(2)+2)= mat_sm5_exp(row,:);
        elseif row==3
            occ_exp_map(nowPose(1),nowPose(2)-2:nowPose(2)+2)=mat_sm5_exp(row,:);
        elseif row==4
            occ_exp_map(nowPose(1)+1,nowPose(2)-2:nowPose(2)+2)=mat_sm5_exp(row,:);
        elseif row==5
            occ_exp_map(nowPose(1)+2,nowPose(2)-2:nowPose(2)+2)=mat_sm5_exp(row,:);
        else
            % do nothing.
        end
    end
end
function [nowPose,vect_sum,vect_sum2,moved_all_flags]=directional_sums(nowPose,occ_exp_map,obs_flag_vector,i)
% calculate directional sums to determine where the robot should prioritize exploration
% changes taking place  on 06/16/22
sum_up = sum(occ_exp_map(nowPose(1)-2,nowPose(2)-1:nowPose(2)+1));
sum_dn = sum(occ_exp_map(nowPose(1)+2,nowPose(2)-1:nowPose(2)+1));
sum_lt = sum(occ_exp_map(nowPose(1)-1:nowPose(1)+1,nowPose(2)-2));
sum_rt = sum(occ_exp_map(nowPose(1)-1:nowPose(1)+1,nowPose(2)+2));
% we need to force the robot to travel in a direction without obstacles...
% if the directional sum is 6 then the expansion model is expanding beyond an obstacle that is located in the sensor model. Therefore, we can use that information to our advantage.
% the i>2 condition prevents the robot from taking an undesired behavhior-travels away from the obstacle-- when the initial robot postion is near or at specific corner.
if i>2
    % @ top left corner:
    % sum_up=6, sum_lt=6, sum_dn=? (should be close to zero,
    % robot is traveling from down to up), sum_rt = ? (desired direction of
    % travel)
    % note: the reason why sum_lt can be either 3 or 6 is because
    %   - 3 corresponds to the obstacle values (three occuppied cells with a value of 1)
    %   - 6 corresponds to values beyond an obstacle and are assigned a value  of 2 (3x2=6)
    if (sum_up==6 && sum_lt==3) || (sum_up==6 && sum_lt==6)
        disp("(sum_up==6 && sum_lt==3) || (sum_up==6 && sum_lt==6)")
        if sum_rt>sum_dn
            sum_rt = sum_rt+10;
        else
            sum_dn= sum_dn+10;
        end
    end
    % @ botttom left corner:
    % sum_lt=6, sum_dn=6, sum_up=?, sum_rt =? (desired direction of travel)
    if (sum_lt==6 && sum_dn==3)||(sum_lt==6 && sum_dn==6)
        disp("(sum_lt==6 && sum_dn==3)||(sum_lt==6 && sum_dn==6)")
        if sum_rt>sum_up
            sum_rt = sum_rt+10;
        else
            sum_up= sum_up+10;
        end
    end
    % @ top right corner:
    % sum_up=6, sum_rt=6 sum_dn=?, sum_lt = ? (desired direction of travel)
    if (sum_up==6 && sum_rt==3) || (sum_up==6 && sum_rt==6)
        disp("(sum_up==6 && sum_rt==3) || (sum_up==6 && sum_rt==6)")
        if sum_lt>sum_dn
            sum_lt = sum_lt+10;
        else
            sum_dn = sum_dn+10;
        end
    end
    % @ bottom right corner:
    % sum_rt=6, sum_dn=6, sum_up=?, sum_lt =? (desired direction of travel)
    if (sum_rt==6 && sum_dn==3) || (sum_rt==6 && sum_dn==6)
        disp("(sum_rt==6 && sum_dn==3) || (sum_rt==6 && sum_dn==6)")
        if sum_lt>sum_up
            sum_lt = sum_lt+10;
        else
            sum_up = sum_up+10;
        end

    end
end
% prevent from going off the grid
% -up-
% goes up, observes obstacle, goes left or right to avoid obstacle
if (sum_up==6 && sum_up>sum_dn && sum_up>sum_lt && sum_up>sum_rt)
    if sum_lt>sum_rt
        sum_lt = sum_lt +10;
    else
        sum_rt = sum_rt +10;
    end
end
% -down-
% goes down, observes obstacle, goes left or right to avoid obstacle

% we only want to do this when the sum_dn = 2+2+2 rather than sum_dn=6
if (sum_dn==6 && sum_dn>sum_up && sum_dn>sum_lt && sum_dn>sum_rt)
    if sum_lt>sum_rt
        sum_lt = sum_lt +10;
    else
        sum_rt = sum_rt +10;
    end
end
% -left-
% goes left, observes obstacle, goes up or down to avoid obstacle
if (sum_lt==6 && sum_lt>sum_up && sum_lt>sum_dn && sum_lt>sum_rt)
    if sum_up>sum_dn
        sum_dn = sum_dn +10;
    else
        sum_up = sum_up +10;
    end
end
% -right-
% goes right, observes obstacle, goes up or down to avoid obstacle
if (sum_rt==6 && sum_rt>sum_up && sum_rt>sum_dn && sum_rt>sum_lt)
    if sum_up>sum_dn
        sum_dn = sum_dn +10;
    else
        sum_up = sum_up +10;
    end
end
% - begin direction correction code -
% if an obstacle is detected on the right hand side, maintain close distance to the obstacle, keep going up until the robot makes close contact with another obstacle at the top
if sum_rt==6 && sum_rt<sum_up && sum_rt<sum_lt % what about sum_dn
    sum_up= sum_up+10;
end
if sum_up==6 && sum_up<sum_lt && sum_up<sum_dn % what about sum_rt???
    sum_lt= sum_lt+10;
end
if sum_lt==6 && sum_lt<sum_rt && sum_lt<sum_dn % what about sum_dn???
    sum_dn= sum_dn+10;
end
if sum_dn==6 && sum_dn<sum_rt && sum_dn<sum_up % what about sum_lt???
    sum_rt= sum_rt+10;
end
% - end direction correction code -
vect_sum = [sum_up sum_dn sum_lt sum_rt]; vect_sum2 = sort(vect_sum,'descend');
fprintf('sum_up: %d; sum_dn: %d; sum_lt: %d; sum_rt: %d \n',sum_up,sum_dn,sum_lt,sum_rt)
% what will happen when there are mutiple directions with the same value, which direction will be chosen? --> make it upward biased for now.
n = 2;
switch n
    case 1 % prioritize [up,down,left,right]
       % removed code for cleaning up...
    case 2 % prioritize [up,left,down,right]
        if vect_sum2(1)==sum_up
            moved_up_flag =1;
            moved_dn_flag =0;
            moved_lt_flag =0;
            moved_rt_flag =0;
            %nowPose = nowPose + [-1 0];
           % disp('moving up')
        elseif vect_sum2(1)==sum_lt
            moved_up_flag =0;
            moved_dn_flag =0;
            moved_lt_flag =1;
            moved_rt_flag =0;
            %nowPose = nowPose+ [0 -1];
            %disp('moving left')
        elseif vect_sum2(1)==sum_dn
            moved_up_flag =0;
            moved_dn_flag =1;
            moved_lt_flag =0;
            moved_rt_flag =0;
            %nowPose = nowPose + [1 0];
            %disp('moving down')
        elseif vect_sum2(1)==sum_rt
            moved_up_flag =0;
            moved_dn_flag =0;
            moved_lt_flag =0;
            moved_rt_flag =1;
            %nowPose = nowPose + [0 1];
            %disp('moving right')
        else
            % do nothing
        end
    otherwise
        disp("invalid selection.")
end
moved_all_flags = [moved_up_flag,moved_dn_flag,moved_lt_flag,moved_rt_flag];
end
function perc_exp = percent_explored(occ_grt_map,occ_ego_map)
mapSize2 = size(occ_grt_map);
x_size = mapSize2(1); 
y_size = mapSize2(2); % redundant
occ_grt_map_int = occ_grt_map(3:x_size-2,3:y_size-2);
occ_ego_map_int = occ_ego_map(3:x_size-2,3:y_size-2);
% compare the two matrices above cell by cell and if the cells are identical then assign a 1 (if different assign 0) to the same location in the occ_perc_mat
occ_perc_mat = zeros(x_size-4,y_size-4);
for row = 1:x_size-4
    for col = 1:y_size-4
        if occ_grt_map_int(row,col)==occ_ego_map_int(row,col)
            occ_perc_mat(row,col)=1;
        else
            occ_perc_mat(row,col)=0;
        end
    end
end
    [~,~,v3]=find(occ_perc_mat==1);
    matched_cells = sum(v3);
    num_cells = row*col;
    perc_exp = (matched_cells/num_cells)*100;
end
end
\end{lstlisting}
