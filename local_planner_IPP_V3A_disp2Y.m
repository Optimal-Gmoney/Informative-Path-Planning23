function [occ_ego_map, return_pose,poses,LP_active,GP_active,egoFigure,map_explored_flag,LP3A_robot_steps] = local_planner_IPP_V3A_disp2Y(nowPose,N, occ_grt_map, occ_ego_map,occ_exp_map,colors,xmax,ymax,total_robot_steps)
poses = zeros(2,N);
poses(:,1) = nowPose;
% grtFigure = figure('Name','ground truth map');
% position_figure(2,3,1)
% %heatmap(occ_grt_map)
% surf(flip(occ_grt_map,1)); view(2);
% colormap(colors);
egoFigure = figure('Name','egocentric map');
position_figure(2,3,2)
%heatmap(occ_ego_map)
%surf(flip(occ_ego_map,1)); view(2);
imagesc(occ_ego_map);
colormap(colors);
expFigure = figure('Name','expansion map');
position_figure(2,3,3)
%heatmap(occ_exp_map)
%surf(flip(occ_exp_map,1)); view(2);
imagesc(occ_exp_map);
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
t1 = zeros(1,N);t2=zeros(1,N);t3=zeros(1,N);t4=zeros(1,N);t5=zeros(1,N);
toc_loop=zeros(1,N);
sum_up = zeros(1,N);sum_dn=zeros(1,N);sum_lt=zeros(1,N);sum_rt=zeros(1,N);
tic_entire_loop=tic; % timing loop completion
% flags keeping track of the stopping criterion
map_explored_flag = 0; zero_direcSums_flag = 0;
pose_oscillation_flag = 0; less3_directSums_flag = 0;
count_steps1 = total_robot_steps;
for i=1:N
%pause the simulation if robot steps is divisible by 330 (12/14/22)
%-----
%     count_steps1 = count_steps1 + 1
%     if mod(count_steps1,330)==0
%         perc_exp = percent_explored(occ_grt_map,occ_ego_map);
%         figure(egoFigure);
%         %heatmap(occ_ego_map)
%         %surf(flip(occ_ego_map,1)); view(2);
%         imagesc(occ_ego_map);
%         colormap(colors);
%         text = sprintf("Robot Location:: X: %4.2f Y: %4.2f @ iteration %d. %% EXP::%4.2f%%",nowPose(2)+0.5,nowPose(1)+0.5,i,perc_exp);
%         title(text)
%         hold on;
%         %plot(nowPose(2)+offset_0p5,nowPose(1)+offset_0p5,'ro')
%         plot(nowPose(2),nowPose(1),'rx')
% 
%         pause()
%     end
%-----
    tic_loop(i)=tic;
    fprintf("Robot Location:: X:%4.2f, Y:%4.2f @ iter. %d \n",nowPose(2)+0.5,nowPose(1)+0.5,i)
    %check to see if the  sensor model occuppies an occupied element in the occupancy matrix
    t1_tic(i) = tic;
    [occ_ego_map, obs_flag_vector1, obs_flag_vector2, obs_flag_vector5]=check_occ_sensor_model(nowPose,occ_grt_map,occ_ego_map);
    t1(i)= toc(t1_tic(i));
    t2_tic(i)=tic;
    perc_exp = percent_explored(occ_grt_map,occ_ego_map);
    t2(i)=toc(t2_tic(i));
    %     pause(0.01);
    figure(egoFigure);
    %heatmap(occ_ego_map)
    %surf(flip(occ_ego_map,1)); view(2);
    imagesc(occ_ego_map);
    colormap(colors);
    text = sprintf("Robot Location:: X: %4.2f Y: %4.2f @ iteration %d. %% EXP::%4.2f%%",nowPose(2)+0.5,nowPose(1)+0.5,i,perc_exp);
    title(text)
    % visualize the robot location
    figure(egoFigure);
    hold on;
    %plot(nowPose(2)+offset_0p5,nowPose(1)+offset_0p5,'ro')
    plot(nowPose(2),nowPose(1),'rx')
    %(added on 11/12/22; Denver, Co w/ NMT Rugby)
    %plot(nonzeros(poses(2,:)),nonzeros(poses(1,:)),'-r') 
    % assign the current occ_ego_map to the occ_exp_map
    occ_exp_map = occ_ego_map;
    %     pause(0.01);
    figure(expFigure);
    %heatmap(occ_exp_map)
    %     surf(flip(occ_exp_map,1)); view(2);
    imagesc(occ_exp_map);
    colormap(colors);
    % returns occ_exp_map and the 5x5 representation of sensor model probably no need to return 5x5 sensor model
    t3_tic(i)=tic;
    [occ_exp_map,mat_sm5_exp]=check_occ_exp_sensor_model(nowPose,occ_exp_map,exp3Figure,colors);
    t3(i)=toc(t3_tic(i));
    % obtain next direction of travel
    t4_tic(i)=tic;
    [nowPose,vect_sum,vect_sum2,moved_all_flags]=directional_sums(nowPose,occ_exp_map,obs_flag_vector1, obs_flag_vector2, obs_flag_vector5, mat_sm5_exp,i);
    % directional sums
    sum_up(i)=vect_sum(1); sum_dn(i)=vect_sum(2);
    sum_lt(i)=vect_sum(3);sum_rt(i)=vect_sum(4);
    % flags indicating movement
    moved_up_flag =moved_all_flags(1);
    moved_dn_flag =moved_all_flags(2);
    moved_lt_flag =moved_all_flags(3);
    moved_rt_flag =moved_all_flags(4);
    t4(i)=toc(t4_tic(i));
    poses(:,i+1)=nowPose; % for plotting poses
    % find difference between posees to find the direction that the robot  moved in; this will help in the inititate_backtrack() function
    diff_poses = poses(:,i+1)'-poses(:,i)';
    t5_tic(i)=tic;
    t5(i)=toc(t5_tic(i));
    % note vect_sum contains [sum_up sum_dn sum_lt sum_rt]
    %     poses(:,i+1)=nowPose; % for plotting poses
    %     pause(0.01);
    figure(expFigure);
    %heatmap(occ_exp_map)
    %surf(flip(occ_exp_map,1)); view(2);
    imagesc(occ_exp_map);
    colormap(colors);
    text = sprintf("Directional Sums:: up: %d; dn: %d; lt: %d; rt: %d @ iteration %d", vect_sum(1),vect_sum(2),vect_sum(3),vect_sum(4),i);
    title(text)
    % plot poses of the robot
    %filled_poses = nonzeros(poses);
    figure(poseFigure);
    set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
    xlim([0 xmax+2+3]) % +2 (border), +3 (equal spacing left and right)
    ylim([0 ymax+2+3])
    hold on;
    plot(poses(2,i)+offset_0p5,poses(1,i)+offset_0p5,'ob')
    text = sprintf("Robot Location:: X: %4.2f Y: %4.2f @ iteration %d", poses(2,i)+offset_0p5,poses(1,i)+offset_0p5,i);
    title(text)
    %pause()
    toc_loop(i)=toc(tic_loop(i));
    %------------------------06/03/22------------------------------
    if moved_up_flag == 1
        direc_indicator = 1;
    elseif moved_dn_flag == 1
        direc_indicator = 2;
    elseif moved_lt_flag == 1
        direc_indicator = 3;
    elseif moved_rt_flag == 1
        direc_indicator = 4;
    else
        direc_indicator = 0; % the robot did not move
    end
    %disp("LP_V3A:: Pose and Directional Sum Info")
    pose_direc_info(i,:) = [poses(2,i)+0.5, poses(1,i)+0.5, vect_sum(1), vect_sum(2),...
        vect_sum(3), vect_sum(4), direc_indicator, i];
    % add driection chosen by robot::
    % sum_up: 1, sum_dn: 2, sum_lt: 3, sum_rt: 4
    %----------------------------------------------------------------------
    % CRITERION FOR STOPPING SIMULATION
    % stopping the program once the map has been explored
    if perc_exp==100
        disp('Congrats, the map has been fully explored.')
        return_pose = poses(:,i)+offset_0p5; % test to see if this works
        % everything in the map has been explored.
        map_explored_flag = 1;
        LP_active = 0; % flag indicates that local planner is deactived
        GP_active = 0; % flat indicates that global planner is actived
        % nonzero robot poses
        LP3A_robot_steps =size(nonzeros(poses(1,:)),1);
    end
    % stopping the program if all directional sums are zero or if an obstacle is detected... (obstacle part needs more work) note (i+1) indexing here... if the robot does not detect any unexplored region, then return the pose one index ahead
    if vect_sum(1)==0 && vect_sum(2)==0 && vect_sum(3)==0 && vect_sum(4)==0
        disp('All directional sums are zero. Local planner simulation stopped. Initiate global planner.')
        return_pose = poses(:,i)+offset_0p5; % test to see if this works
        LP_active = 0; % flag indicates that local planner is deactived
        GP_active = 1 ; % flat indicates that global planner is actived
        zero_direcSums_flag = 1; % flag is set to true
        % nonzero robot poses
        LP3A_robot_steps =size(nonzeros(poses(1,:)),1);
        % obstacle detected, might need to change the value of 3
    elseif vect_sum(1)<=4 && vect_sum(2)<=4 && vect_sum(3)<=4 && vect_sum(4)<=4
        disp('All directional sums are less than 4. Local planner simulation stopped. Initiate global planner.')
        return_pose = poses(:,i)+offset_0p5; % test to see if this works
        LP_active = 0; % flag indicates that local planner is deactived
        GP_active = 1 ; % flag indicates that global planner is actived
        less3_directSums_flag = 1; % flag is set to true
        % nonzero robot poses
        LP3A_robot_steps =size(nonzeros(poses(1,:)),1);
    else
        LP_active = 1; % flag indicates that local planner is activated
        GP_active = 0; % flat indicates that global planner is deactived
        % nonzero robot poses
        LP3A_robot_steps =size(nonzeros(poses(1,:)),1);
    end
    % stopping the program if robot location osicillates between two
    % locations
    prev4_poses_lin = zeros(1,4); % might cause an error if not 1x4 vector
    if mod(i,4)==0
        mapSize = size(occ_ego_map);
        prev4_poses = poses(:,i-3:i); % important change.
        for jj = 1:size(prev4_poses,2)
            prev4_poses_lin(jj) = sub2ind(mapSize,prev4_poses(1,jj),prev4_poses(2,jj)); % important change
        end
        % comparing third element in last 4 poses to the remainder 3
        check3_poses1 = [prev4_poses_lin(:,1:2) prev4_poses_lin(:,4)];
        % comparing fourht element in the last 4 poses to the remainder 3
        check3_poses2 = [prev4_poses_lin(:,1:2) prev4_poses_lin(:,3)];
        if ismember(prev4_poses_lin(:,3),check3_poses1) && ismember(prev4_poses_lin(:,4),check3_poses2)
            disp('Robot location is osciallating between two locations. Stop local planner and initiate global planner.')
            return_pose = poses(:,i)+offset_0p5;
            LP_active = 0; % flag indicates that local planner is deactived
            GP_active = 1 ; % flat indicates that global planner is actived
            pose_oscillation_flag = 1; % flag is set to true
            % nonzero robot poses
            LP3A_robot_steps =size(nonzeros(poses(1,:)),1);
        end
    end
    if map_explored_flag ==1 || zero_direcSums_flag ==1 || pose_oscillation_flag ==1 || less3_directSums_flag ==1
        disp("LP_V3A:: Pose and Directional Sum Info")
        disp(pose_direc_info)
        %         pause()
        break;
        % nonzero robot poses
        LP3A_robot_steps =size(nonzeros(poses(1,:)),1);
    end
    % close figures
    %close(figure(exp3Figure));close(figure(egoFigure));close(figure(expFigure));
    %close all
end 
% num = size(nonzerso(toc_loop));
% avg_loop_time =sum(nonzeros(toc_loop))/num;
toc_entire_loop = toc(tic_entire_loop);
% function will check if the sensor model in the occ_exp_map occupies an occupied cell in the occ_grt_map it will return which cells are accopied so the robot does not travel in that direction
function [occ_ego_map, obs_flag_vector1,obs_flag_vector2,obs_flag_vector5]=check_occ_sensor_model(nowPose,occ_grt_map,occ_ego_map)
% to make it easy, why not extract the values of the group of cells in the occ_grt_map that the sensor model occupies in the occ_exp_map, then check which cells are occupied (value of 1) and id them. Transfer the gained map information to the occ_ego_map.
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
%------------------------------------------------------------------
[row4,col4,~] = find(mat_sm3_ego==1);
obs_rt_flag_sum3 = 0; obs_rt_flag_sum2 = 0; obs_rt_mid_flag = 0;
obs_lt_flag_sum3 = 0; obs_lt_flag_sum2 = 0; obs_lt_mid_flag = 0;
obs_dn_flag_sum3 = 0; obs_dn_flag_sum2 = 0; obs_dn_mid_flag = 0;
obs_up_flag_sum3 = 0; obs_up_flag_sum2 = 0; obs_up_mid_flag = 0;
% only display the sensor model matrix if there exist obstacles
if isempty(row4)==0
    disp("sensor model matrix")
    disp(mat_sm3_ego)
end
% determien the value of each cell independently and then add
% them together.
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
    obs_rt_flag_sum3 = 1;
    disp("SUM3:: obstacle to the RIGHT of robot")
elseif sum_obs_rt == 2
    obs_rt_flag_sum2 = 1;
    disp("SUM2:: obstacle to the RIGHT of robot")
else
    % do nothing
end
if sum_obs_lt == 3
    obs_lt_flag_sum3 = 1;
    disp("SUM3:: obstacle to the LEFT of robot")
elseif sum_obs_lt == 2
    obs_lt_flag_sum2 = 1;
%             disp("SUM2:: obstacle to the LEFT of robot")
else
    % do nothing
end
if sum_obs_dn == 3
    obs_dn_flag_sum3 = 1;
    disp("SUM3:: obstacle to the DOWN of robot")
elseif sum_obs_dn == 2
    obs_dn_flag_sum2 = 1;
    disp("SUM2:: obstacle to the DOWN of robot")
else
    % do nothing
end
if sum_obs_up == 3
    obs_up_flag_sum3 = 1;
    disp("SUM3:: obstacle to the UP of robot")
elseif sum_obs_up == 2
    obs_up_flag_sum2 = 1;
    disp("obstacle to the UP of robot")
else
    % do nothing
end
% need to check if the middle cell in all directions is occupied (value of 1). If it is occupied then do not let the robot travel using that cell.
% flags go here... (06/28/22)
sum_sm_up_lt2_flag = 0; sum_sm_up_rt2_flag = 0;
sum_sm_dn_lt2_flag = 0; sum_sm_dn_rt2_flag = 0;
sum_sm_lt_up2_flag = 0; sum_sm_lt_dn2_flag = 0;
sum_sm_rt_up2_flag = 0; sum_sm_rt_dn2_flag = 0;
if top_mid == 1 && obs_up_flag_sum3 ~= 1 % check to see if sum == 2, information will be used to guide the robot
    if obs_up_flag_sum2 == 1 && top_lt_corner == 1
        sum_sm_up_lt2_flag = 1;
        disp('T071122_c1:: obs_up_flag_sum2 == 1 && top_lt_corner == 1')
    elseif obs_up_flag_sum2 == 1 && top_rt_corner == 1
        sum_sm_up_rt2_flag = 1;
        disp('T071122_c1:: obs_up_flag_sum2 == 1 && top_rt_corner == 1')
    else
        % obs_up_mid_flag = 1;
        % disp("WARNING: obstacle located at top_mid cell")
    end
end
if lt_mid == 1 && obs_lt_flag_sum3 ~= 1
    if obs_lt_flag_sum2 == 1 && top_lt_corner == 1
        sum_sm_lt_up2_flag = 1;
        disp('T071122_c1:: obs_lt_flag_sum2 == 1 && top_lt_corner == 1')
    elseif obs_lt_flag_sum2 ==1 && bot_lt_corner == 1
        sum_sm_lt_dn2_flag = 1;
        disp('T071122_c1:: obs_lt_flag_sum2 ==1 && bot_lt_corner == 1')
    else
        % obs_lt_mid_flag = 1;
        % disp("WARNING: obstacle located at lt_mid cell")
    end
end
if rt_mid == 1 && obs_rt_flag_sum3 ~= 1
    if obs_rt_flag_sum2 == 1 && top_rt_corner == 1
        sum_sm_rt_up2_flag = 1;
        disp('T071122_c1:: obs_rt_flag_sum2 == 1 && top_rt_corner == 1')
    elseif obs_rt_flag_sum2 == 1 && bot_rt_corner ==1
        sum_sm_rt_dn2_flag = 1;
        disp('T071122_c1:: obs_rt_flag_sum2 == 1 && bot_rt_corner ==1')
    else
        % obs_rt_mid_flag = 1;
        % disp("WARNING: obstacle located at rt_mid cell")
    end
end
if bot_mid == 1 && obs_dn_flag_sum3 ~= 1
    if obs_dn_flag_sum2 == 1 && bot_lt_corner == 1
        sum_sm_dn_lt2_flag = 1;
        disp('T071122_c1:: obs_dn_flag_sum2 == 1 && bot_lt_corner == 1')
    elseif obs_dn_flag_sum2 == 1 && bot_rt_corner == 1
        sum_sm_dn_rt2_flag = 1;
        disp('T071122_c1:: obs_dn_flag_sum2 == 1 && bot_rt_corner == 1')
    else
        % obs_dn_mid_flag = 1;
        % disp("WARNING: obstacle located at bot_mid cell")
    end
end
obs_flag_vector1 = [obs_up_flag_sum3 top_mid;
    obs_dn_flag_sum3 bot_mid;
    obs_lt_flag_sum3 lt_mid;
    obs_rt_flag_sum3 rt_mid];
% quick_here_obs_flag_vector2
obs_flag_vector2 = [sum_sm_up_lt2_flag sum_sm_up_rt2_flag;
    sum_sm_dn_lt2_flag sum_sm_dn_rt2_flag;
    sum_sm_lt_up2_flag sum_sm_lt_dn2_flag;
    sum_sm_rt_up2_flag sum_sm_rt_dn2_flag];
obs_flag_vector5 = [top_rt_corner bot_rt_corner top_lt_corner bot_rt_corner];
%------------------------------------------------------------------
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
% look identify the cells that have a value of 1 in mat_sm5_exp, then check
% the surrounding cells: change cells that are in direct contact with an
% occupied cell from 0.5 to 2. If not in direct contact then change from
% 0.5 to 6.
[row,col,v]=find(mat_sm5_exp==1);
for n=1:length(row)
    % do nothing.
    % consider the case when it is row 1 then we cannot subtract 1 bc of
    % indexing issue, i.e., no row 0
    % fprintf("row: %d; col: %d \n",row(n),col(n))
    %|| row(n)==7 || col(n)==7 || col(n)==1
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
figure(exp3Figure);
%         heatmap(mat_sm5_exp,'CellLabelColor','none')
%surf(flip(mat_sm5_exp,1)); view(2);
imagesc(mat_sm5_exp);
colormap(colors);
% the code commented below displays the sensor model for the pose of interest
% quick_here_sm_pose
% note: nowPose(1) = Y, nowPose(2) = X
% #EZPOSE
if nowPose(1)== 20 && nowPose(2)== 9
    disp("sensor model matrix under pose")
    mat_sm5_exp
    pause()
end
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
function [nowPose,vect_sum,vect_sum2,moved_all_flags]=directional_sums(nowPose,occ_exp_map,obs_flag_vector1,obs_flag_vector2,obs_flag_vector5,mat_sm5_exp,i)
% calculate directional sums to determine where the robot should prioritize exploration
sum_up = sum(occ_exp_map(nowPose(1)-2,nowPose(2)-1:nowPose(2)+1));
sum_up_lt_cell = occ_exp_map(nowPose(1)-2, nowPose(2)-1);
sum_up_rt_cell = occ_exp_map(nowPose(1)-2, nowPose(2)+1);
sum_dn = sum(occ_exp_map(nowPose(1)+2,nowPose(2)-1:nowPose(2)+1));
sum_dn_lt_cell = occ_exp_map(nowPose(1)+2,nowPose(2)-1);
sum_dn_rt_cell = occ_exp_map(nowPose(1)+2,nowPose(2)+1);
sum_lt = sum(occ_exp_map(nowPose(1)-1:nowPose(1)+1,nowPose(2)-2));
sum_lt_dn_cell = occ_exp_map(nowPose(1)+1,nowPose(2)-2);
sum_lt_up_cell = occ_exp_map(nowPose(1)-1,nowPose(2)-2);
sum_rt = sum(occ_exp_map(nowPose(1)-1:nowPose(1)+1,nowPose(2)+2));
sum_rt_dn_cell = occ_exp_map(nowPose(1)+1,nowPose(2)+2);
sum_rt_up_cell = occ_exp_map(nowPose(1)-1,nowPose(2)+2);

obs_flag_vector3 = [sum_up_lt_cell sum_up_rt_cell;
    sum_dn_lt_cell sum_dn_rt_cell;
    sum_lt_dn_cell sum_lt_up_cell;
    sum_rt_dn_cell sum_rt_up_cell];
% getting the corner cells from the sensor model for obstacle avoidance 
sm_top_rt_corner = obs_flag_vector5(1); sm_top_lt_corner = obs_flag_vector5(3);
sm_bot_rt_corner = obs_flag_vector5(2); sm_bot_lt_corner = obs_flag_vector5(4);
% #DH4
sm5_top_lt_cell = mat_sm5_exp(1,1); sm5_top_rt_cell = mat_sm5_exp(1,5);
sm5_bot_lt_cell = mat_sm5_exp(5,1); sm5_bot_rt_cell = mat_sm5_exp(5,5);
% extracting::
% check cells one cell away from robot for obstacles to aid in
% avoiding obstacles
% note: sm = sensor model 3x3 matrix w/ robot location being the robot
%                 |1*  0  1*|
%                 |0   R  0 |
%                 |1*  0  1*|
% note: R = robot position, * indicates the cells that are  extracted below
top_lt_cell_sm = occ_exp_map(nowPose(1)-1,nowPose(2)-1);
top_rt_cell_sm = occ_exp_map(nowPose(1)-1,nowPose(2)+1);
bot_lt_cell_sm = occ_exp_map(nowPose(1)+1,nowPose(2)-1);
bot_rt_cell_sm = occ_exp_map(nowPose(1)+1,nowPose(2)+1);
% We need to force the robot to travel in a direction without obstacles...
% if the directional sum is 6 then the expansion model is expanding beyond an obstacle that is located in the sensor model. Therefore, we can use that information to our advantage.
% the i>2 condition prevents the robot from taking an undesired behavhior-- travels away from the obstacle-- when the initial robot postion is near or at specific corner.
if i>2
    % @ top left corner:
    % sum_up=6, sum_lt=6, sum_dn=? (should be close to zero,
    % robot is traveling from down to up), sum_rt = ? (desired direction of
    % travel)
    % note: the reason why sum_lt can be either 3 or 6 is because
    %- 3 corresponds to the obstacle values (three occuppied cells with a value of 1)
    %- 6 corresponds to values beyond an obstacle and are assigned a value of 2 (3x2=6)
    if ((sum_up==6 && sum_lt==3) || (sum_up==6 && sum_lt==6)) && obs_flag_vector1(4,1) ~= 1 && obs_flag_vector1(2,1) ~= 1 &&...
            ((sum_up_lt_cell == 2 || sum_up_rt_cell == 2) && sum_up == 6) && ((sum_lt_dn_cell == 2 || sum_lt_up_cell == 2) && sum_lt == 6)
        disp("T070122_a1:: (sum_up==6 && sum_lt==3) || (sum_up==6 && sum_lt==6)")
        if sum_rt>sum_dn && sum_rt ~=1 && sum_rt ~=3 && sum_rt ~=6
            sum_rt = sum_rt+10;
        elseif sum_dn>sum_rt && sum_dn ~=1 && sum_dn ~=3 && sum_dn ~=6
            sum_dn= sum_dn+10;
        else
            disp("T070122_a1:: Warning LP: top left corner error.")
        end
    end
    % @ botttom left corner:
    % sum_lt=6, sum_dn=6, sum_up=?, sum_rt =? (desired direction of travel)
    if ((sum_lt==6 && sum_dn==3)||(sum_lt==6 && sum_dn==6)) && obs_flag_vector1(4,1) ~= 1 && obs_flag_vector1(1,1) ~= 1 && ...
            ((sum_dn_lt_cell == 2 || sum_dn_rt_cell == 2) && sum_dn == 6) && ((sum_lt_dn_cell == 2 || sum_lt_up_cell == 2) && sum_lt == 6)
        disp("(T070122_a2:: sum_lt==6 && sum_dn==3)||(sum_lt==6 && sum_dn==6)")
        if sum_rt>sum_up && sum_rt~=1 && sum_rt~=3 && sum_rt~=6
            sum_rt = sum_rt+10;
        elseif sum_up>sum_rt && sum_up~=1 && sum_up~=3 && sum_up~=6
            sum_up= sum_up+10;
        else
            disp("T070122_a2:: Warning LP: bottom left corner error.")
        end
    end
    % @ top right corner:
    % sum_up=6, sum_rt=6 sum_dn=?, sum_lt = ? (desired direction of travel)
    if ((sum_up==6 && sum_rt==3) || (sum_up==6 && sum_rt==6)) && obs_flag_vector1(3,1) ~= 1 && obs_flag_vector1(2,1) ~= 1 && ...
            ((sum_up_lt_cell == 2 || sum_up_rt_cell == 2) && sum_up == 6) && ((sum_rt_dn_cell == 2 || sum_rt_up_cell == 2) && sum_rt == 6)
        disp("T070122_a3:: (sum_up==6 && sum_rt==3) || (sum_up==6 && sum_rt==6)")
        if sum_lt>sum_dn && sum_lt ~=1 && sum_lt ~=3 && sum_lt ~=6
            sum_lt = sum_lt+10;
        elseif sum_dn>sum_lt && sum_dn ~=1 && sum_dn ~=3 && sum_dn ~=6
            sum_dn = sum_dn+10;
        else
            disp("T070122_a3:: Warning LP: top right corner error.")
        end
    end
    % @ bottom right corner:
    % sum_rt=6, sum_dn=6, sum_up=?, sum_lt =? (desired direction of travel)
    if ((sum_rt==6 && sum_dn==3) || (sum_rt==6 && sum_dn==6)) && obs_flag_vector1(3,1) ~= 1 && obs_flag_vector1(1,1) ~= 1 && ...
            ((sum_dn_lt_cell == 2 || sum_dn_rt_cell == 2) && sum_dn == 6) && ((sum_rt_dn_cell == 2 || sum_rt_up_cell == 2) && sum_rt == 6)
        disp("T070122_a4:: (sum_rt==6 && sum_dn==3) || (sum_rt==6 && sum_dn==6)")
        if sum_lt>sum_up && sum_lt ~=1 && sum_lt ~=3 && sum_lt ~=6
            sum_lt = sum_lt+10;
        elseif sum_up>sum_lt && sum_up ~=1 && sum_up ~=3 && sum_up ~=6
            sum_up = sum_up+10;
        else
            disp("T070122_a4:: Warning LP: bottom right corner error.")
        end
    end
end
% prevent from going off the grid
% -up-
% goes up, observes obstacle, goes left or right to avoid obstacle
if (sum_up==6 && (sum_up_lt_cell==2 || sum_up_rt_cell==2) && sum_up>sum_dn && sum_up>sum_lt && sum_up>sum_rt)
    if sum_lt>sum_rt && sum_up_rt_cell == 1 && sum_lt>3
        sum_lt = sum_lt +10;
        disp("T062922_up:: sum_lt = sum_lt +10")
    elseif sum_rt>sum_lt && sum_up_lt_cell == 1 && sum_lt>3
        sum_rt = sum_rt +10;
        disp("T062922_up:: sum_rt = sum_rt +10")
    else
        disp("T062922_up:: Warning LP: UP direction in preventing robot from going of grid.")
    end
end
% -down-
% goes down, observes obstacle, goes left or right to avoid obstacle
% we only want to do this when the sum_dn = 2+2+2 rather than sum_dn=6
if (sum_dn==6 &&(sum_dn_lt_cell==2||sum_dn_rt_cell==2) && sum_dn>sum_up && sum_dn>sum_lt && sum_dn>sum_rt)
    if sum_lt>sum_rt && sum_dn_rt_cell == 1 && sum_lt>3
        sum_lt = sum_lt +10;
        disp("T062922_dn:: sum_lt = sum_lt +10")
    elseif sum_rt>sum_lt && sum_dn_lt_cell == 1 && sum_rt>3
        sum_rt = sum_rt +10;
        disp("T062922_dn:: sum_rt = sum_rt +10")
    else
        disp("T062922_dn:: Warning LP: DN direction in preventing robot from going of grid.")
    end
end
% -left-
% goes left, observes obstacle, goes up or down to avoid obstacle
if (sum_lt==6 && (sum_lt_up_cell==2||sum_lt_dn_cell==2) && sum_lt>sum_up && sum_lt>sum_dn && sum_lt>sum_rt)
    if sum_up>sum_dn && sum_dn_lt_cell ==1 && sum_up>3
        sum_up = sum_up +10;
        disp("T062922_lt:: sum_up = sum_up +10")
    elseif sum_dn>sum_up && sum_up_lt_cell == 1 && sum_dn>3
        sum_dn = sum_dn +10;
        disp("T062922_lt:: sum_dn = sum_dn +10")
    else
        disp("T062922_dn:: Warning LP: LT direction in preventing robot from going of grid.")
    end
end
% -right-
% goes right, observes obstacle, goes up or down to avoid obstacle
if (sum_rt==6 &&(sum_rt_up_cell==2||sum_rt_dn_cell==2) && sum_rt>sum_up && sum_rt>sum_dn && sum_rt>sum_lt)
    if sum_up>sum_dn && sum_dn_rt_cell == 1 && sum_up>3
        sum_up = sum_up +10;
        disp("T062922_rt:: sum_up = sum_up +10")
    elseif sum_dn>sum_up && sum_up_rt_cell == 1 && sum_dn>3
        sum_dn = sum_dn +10;
        disp("T062922_rt:: sum_dn = sum_dn +10")
    else
        disp("T062922_rt:: Warning LP: RT direction in preventing robot from going of grid.")
    end
end
% - begin direction correction code -
% if an obstacle is detected on the right hand side, maintain close distance to the obstacle, keep going up until the robot makes close contact with another obstacle at the top
if sum_rt==6 && sum_rt<sum_up && sum_rt<sum_lt && sum_up>3 && obs_flag_vector1(1,2) ~=1 && obs_flag_vector1(1,1) ~=1 % what about sum_dn
    sum_up= sum_up+10;
    disp('INFO: right wall detected... following')
end
if sum_up==6 && sum_up<sum_lt && sum_up<sum_dn && sum_lt >3 && obs_flag_vector1(3,2) ~=1 && obs_flag_vector1(3,1) ~=1 
    sum_lt= sum_lt+10;
    disp('INFO: top wall detected... following')
end
if sum_lt==6 && sum_lt<sum_rt && sum_lt<sum_dn && sum_dn>3 && obs_flag_vector1(2,2) ~=1 && obs_flag_vector1(2,1) ~=1  % what about sum_dn???
    sum_dn= sum_dn+10;
    disp('INFO: left wall detected... following')
end
if sum_dn==6 && sum_dn<sum_rt && sum_dn<sum_up && sum_rt>3 && obs_flag_vector1(4,2) ~=1 && obs_flag_vector1(4,1) ~=1  % what about sum_lt???
    sum_rt= sum_rt+10;
    disp('INFO: bottom wall detected... following')
end
% - end direction correction code -
vect_sum = [sum_up sum_dn sum_lt sum_rt]; vect_sum2 = sort(vect_sum,'descend');
fprintf('sum_up: %d; sum_dn: %d; sum_lt: %d; sum_rt: %d \n',sum_up,sum_dn,sum_lt,sum_rt)

% what will happen when there are mutiple directions with the same value, which direction will be chosen? --> make it upward-biased fornow.
% quick_here_obst_flags
obs_up_flag = obs_flag_vector1(1,1); obs_up_mid_flag = obs_flag_vector1(1,2);
obs_dn_flag = obs_flag_vector1(2,1); obs_dn_mid_flag = obs_flag_vector1(2,2);
obs_lt_flag = obs_flag_vector1(3,1); obs_lt_mid_flag = obs_flag_vector1(3,2);
obs_rt_flag = obs_flag_vector1(4,1); obs_rt_mid_flag = obs_flag_vector1(4,2);
obs_flag_vector2
sum_sm_up_lt2_flag = obs_flag_vector2(1,1); sum_sm_up_rt2_flag = obs_flag_vector2(1,2);
sum_sm_dn_lt2_flag = obs_flag_vector2(2,1); sum_sm_dn_rt2_flag = obs_flag_vector2(2,2);
sum_sm_lt_up2_flag = obs_flag_vector2(3,1); sum_sm_lt_dn2_flag = obs_flag_vector2(3,2);
sum_sm_rt_up2_flag = obs_flag_vector2(4,1); sum_sm_rt_dn2_flag = obs_flag_vector2(4,2);
%-----------------------updated 06/18/22---------------------------
all_time_poses(i,1) = nowPose(1); all_time_poses(i,2) = nowPose(2);
no_dir_move_flag = 0;
if i>2
    % extracting the previous direction the robot moved from moved_all_flags
    moved_all_flags = [moved_up_flag,moved_dn_flag,moved_lt_flag,moved_rt_flag];
    % note: the reason why the directional_sum must be greater than 6 is to avoid obstacles ... might need to edit the value later.
    % checking to see if the previous and current pose are the same below to prevent osciallting behavior, therefore preventing the global planner to kick in.
    test_values1 = [8, 10, 14];
    test_values2 = [0, 6, 9, 12];
    %test_values23 = [14];
    % olt = other less than; obst = obstacle
    % test_values_1 flags
    sum_up_14_olt6_flag = 0; sum_dn_14_olt6_flag = 0;
    sum_lt_14_olt6_flag = 0; sum_rt_14_olt6_flag = 0;
    % test_values2 flags
    sum_up_14_obst_flag = 0; sum_dn_14_obst_flag = 0;
    sum_lt_14_obst_flag = 0; sum_rt_14_obst_flag = 0;
    % vect_sum = [sum_up sum_dn sum_lt sum_rt]
    test_value4 = 6;
    if sum(ismember(test_values1,vect_sum(1)))==1 && vect_sum(2)<test_value4 && vect_sum(3)<test_value4 && vect_sum(4)<test_value4
        disp('testing: sum_up_14_olt6_flag')
        sum_up_14_olt6_flag = 1;
    elseif sum(ismember(test_values1,vect_sum(2)))==1 && vect_sum(1)<test_value4 && vect_sum(3)<test_value4 && vect_sum(4)<test_value4 % change this requirement...
        disp('testing: sum_dn_14_olt6_flag')
        sum_dn_14_olt6_flag = 1;
    elseif sum(ismember(test_values1,vect_sum(3)))==1 && vect_sum(2)<test_value4 && vect_sum(1)<test_value4 && vect_sum(4)<test_value4
        disp('testing: sum_lt_14_olt6_flag')
        sum_lt_14_olt6_flag = 1;
    elseif sum(ismember(test_values1,vect_sum(4)))==1 && vect_sum(2)<test_value4 && vect_sum(3)<test_value4 && vect_sum(1)<test_value4
        disp('testing: sum_rt_14_olt6_flag')
        sum_rt_14_olt6_flag = 1;
    else
    % do nothing
        disp('INFO: sum_<direction>_14_olt6_flags did not execute')
    end
    test_values5 = [10 14 20];
    sum_dir_20_flag = 0;
    if sum(ismember(vect_sum,test_values5)) ==1
        sum_dir_20_flag = 1;
    end
    % vect_sum = [sum_up sum_dn sum_lt sum_rt]
    % force the robot to travel close to the obstacle so that it is detected.
    if (vect_sum(1) == 14 || vect_sum(1) == 10) && (sum(ismember(test_values2,vect_sum(2)))==1|| sum(ismember(test_values2,vect_sum(3)))==1 || sum(ismember(test_values2,vect_sum(4)))==1)% && sum_dir_18_flag ==0
         disp('testing: sum_up_14_obst_flag')
        sum_up_14_obst_flag = 1;
    elseif (vect_sum(2) == 14 || vect_sum(2) == 10) && (sum(ismember(test_values2,vect_sum(1)))==1 || sum(ismember(test_values2,vect_sum(3)))==1 || sum(ismember(test_values2,vect_sum(4)))==1) %&& sum_dir_18_flag ==0
         disp('testing: sum_dn_14_obst_flag')
        sum_dn_14_obst_flag = 1;
    elseif (vect_sum(3) == 14 || vect_sum(3) == 10) && (sum(ismember(test_values2,vect_sum(1)))==1 || sum(ismember(test_values2,vect_sum(2)))==1 || sum(ismember(test_values2,vect_sum(4)))==1) %&& sum_dir_18_flag ==0
         disp('testing: sum_lt_14_obst_flag')
        sum_lt_14_obst_flag = 1;
    elseif (vect_sum(4) == 14 || vect_sum(4) == 10) && (sum(ismember(test_values2,vect_sum(1)))==1 || sum(ismember(test_values2,vect_sum(2)))==1 || sum(ismember(test_values2,vect_sum(3)))==1) %&& sum_dir_18_flag ==0
         disp('testing: sum_rt_14_obst_flag')
        sum_rt_14_obst_flag = 1;
    else
        % do nothing
    end
    % 06/24/22 this will check if one of the direction sums is 14 the intention for these four flags is to prevent the robot from traveling a different direction when that that direction has a higher directional sum (most likely 18) than 14.
    sum_up_14_obst_vect = [sum_dn_14_obst_flag ...
        sum_lt_14_obst_flag sum_rt_14_obst_flag];  %% quick_here_890
    sum_up_14_obst_flag2s = sum(sum_up_14_obst_vect);
    sum_dn_14_obst_vect = [sum_up_14_obst_flag ...
        sum_lt_14_obst_flag sum_rt_14_obst_flag];
    sum_dn_14_obst_flag2s = sum(sum_dn_14_obst_vect);
    sum_lt_14_obst_vect = [sum_up_14_obst_flag sum_dn_14_obst_flag ...
        sum_rt_14_obst_flag];
    sum_lt_14_obst_flag2s = sum(sum_lt_14_obst_vect);
    sum_rt_14_obst_vect = [sum_up_14_obst_flag sum_dn_14_obst_flag ...
        sum_lt_14_obst_flag];
    sum_rt_14_obst_flag2s = sum(sum_rt_14_obst_vect);
    % robot previously moved up, continue moving up
    if moved_all_flags(1) == 1 && vect_sum(1) > 10 && vect_sum(1) ~= 14 && sum_up_14_olt6_flag ==0 && ...
            (all_time_poses(i-1,1) ~= all_time_poses(i,1)) && ...
            (all_time_poses(i-1,2) ~= all_time_poses(i,2)) && sum_up_14_obst_flag ~=1
        if (sum(obs_flag_vector2(:,1))>=1 || sum(obs_flag_vector2(:,2))>=1) && sum_dir_20_flag ~=1 && obs_up_mid_flag == 1
            % calling function
            disp('T071622:: -up condition-')
            nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); %%% bazinga.
        else
            nowPose = nowPose + [-1 0];
            disp('Robot continually moving up.')
        end
        % robot previously moved dn, continue moving dn
    elseif moved_all_flags(2) == 1 && vect_sum(2) > 10 && vect_sum(2) ~= 14 && sum_dn_14_olt6_flag == 0 && ...
            (all_time_poses(i-1,1) ~= all_time_poses(i,1)) && ...
            (all_time_poses(i-1,2) ~= all_time_poses(i,2)) && sum_dn_14_obst_flag ~=1
        if (sum(obs_flag_vector2(:,1))>=1 || sum(obs_flag_vector2(:,2))>=1) && sum_dir_20_flag ==1 && obs_dn_mid_flag ==1
            % calling function
            disp('T071622:: -dn condition-')
            nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); %%% bazinga.
        else
            nowPose = nowPose + [1 0];
            disp('Robot continually moving down.')
        end
        % robot previously moved lt, continue moving lt
    elseif moved_all_flags(3) == 1 && vect_sum(3) > 10 && vect_sum(3) ~= 14 && sum_lt_14_olt6_flag == 0 &&...
            (all_time_poses(i-1,1) ~= all_time_poses(i,1)) && ...
            (all_time_poses(i-1,2) ~= all_time_poses(i,2)) && sum_lt_14_obst_flag ~=1
        nowPose = nowPose+ [0 -1];
        if (sum(obs_flag_vector2(:,1))>=1 || sum(obs_flag_vector2(:,2))>=1) && sum_dir_20_flag ~=1 && obs_lt_mid_flag == 1
            % calling function
            disp('T071622:: -lt condition-')
            nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); %%% bazinga.
        else
            disp('Robot continually moving left.')
        end
        % robot previously moved rt, continue moving rt
    elseif moved_all_flags(4) == 1 && vect_sum(4) > 10 && vect_sum(4) ~= 14 && sum_rt_14_olt6_flag == 0 && ...
            (all_time_poses(i-1,1) ~= all_time_poses(i,1)) && ...
            (all_time_poses(i-1,2) ~= all_time_poses(i,2)) && sum_rt_14_obst_flag ~=1
        if (sum(obs_flag_vector2(:,1))>=1 || sum(obs_flag_vector2(:,2))>=1) && sum_dir_20_flag ~=1 && obs_rt_mid_flag ==1
            % calling function
            disp('T071622:: -rt condition-')
            nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); %%% bazinga.
        else
            nowPose = nowPose + [0 1];
            disp('Robot continually moving right.')
        end
    else
       % disp("INFO: Robot continual directional movement stopped. Choosing higher directional sum.")
        if vect_sum2(1)==sum_up && obs_up_flag ~= 1 %&& sum_up_14_obst_flag2 ~=1
       % check if other obstacle flags have been triggered if they have then move in that direction instead of up.
            if sum_up_14_obst_flag2s == 1 || sum_up_14_obst_flag == 1
                if (sum(obs_flag_vector2(:,1))>=1 || sum(obs_flag_vector2(:,2))>=1) && sum_dir_20_flag ~=1 && obs_up_mid_flag ==1 
                    % calling function
                    disp('T070122_c1:: avoid_obst_dir2() executed')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); %%% bazinga.
                elseif sum_up_14_obst_vect(1) == 1 && obs_dn_mid_flag ~=1 % move down
                    if obs_dn_mid_flag == 1
                        disp('T070122_c1:: moving down-<lt/rt> / sum_up_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);
                    else
                        moved_up_flag =0; moved_dn_flag =1; moved_lt_flag =0; moved_rt_flag =0;
                        nowPose = nowPose + [1 0];
                        disp('T070122_c1:: moving down / sum_up_14_obst_flag2s')
                    end
                elseif sum_up_14_obst_vect(2) == 1 % move left
                    if obs_lt_mid_flag == 1
                        disp('T070122_c1:: moving left-<up/dn> / sum_up_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);
                    else
                        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =1; moved_rt_flag =0;
                        nowPose = nowPose + [0 -1];
                        disp('T070122_c1:: moving left / sum_up_14_obst_flag2s')
                    end
                elseif sum_up_14_obst_vect(3) == 1 % move right
                    if obs_rt_mid_flag ==1
                        disp('T070122_c1:: mov6ing right-<up/dn> / sum_up_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); 
                    else
                        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =1;
                        nowPose = nowPose + [0 1];
                        disp('T070122_c1:: moving right / sum_up_14_obst_flag2s')
                    end
                elseif sum_up_14_obst_flag == 1
                    if sum_up > 6 && obs_up_flag ~=1 && obs_up_mid_flag ~= 1
                        moved_up_flag =1; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
                        nowPose = nowPose + [-1 0];
                        disp('T070122_c1:: moving up / sum_up_14_obst_flag2s')
                    elseif sum_up > 6 && obs_up_mid_flag ==1
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);
                        disp('T070122_c1:: testing some code <shizzle2>')
                    else
                        disp('T070122_c1:: testing some code <shizzle>')
                    end
                elseif (obs_rt_mid_flag == 1|| obs_lt_mid_flag == 1 || obs_up_mid_flag == 1 || obs_dn_mid_flag==1) %#3KF
                    disp('T071822:: avoid_obst_dir5() executed')
                    nowPose=avoid_obst_dir5(mat_sm5_exp, obs_flag_vector3,nowPose);
                else
                    % do nothing.
                    disp('T070122_c1:: INFO: X_up_X')
                end
            elseif ((sum_up_14_olt6_flag == 1 && (sum_up_lt_cell == 6 || sum_up_lt_cell == 2) && sum_up_rt_cell <= 6 && top_lt_cell_sm ~=1)) || sum_sm_up_rt2_flag ==1 && sm_top_lt_corner ~=1
                nowPose = nowPose + [-1 -1];
                disp('T070122_c1:: moving up-lt')
                %pause()
            elseif ((sum_up_14_olt6_flag == 1 && (sum_up_rt_cell ==6 || sum_up_rt_cell == 2) && sum_up_lt_cell <= 6 && top_rt_cell_sm ~=1) || sum_sm_up_lt2_flag ==1) && sm_top_rt_corner ~=1
                nowPose = nowPose + [-1 1];
                disp('T070122_c1:: moving up-rt')
            elseif obs_up_mid_flag ~=1
                moved_up_flag =1; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
                nowPose = nowPose + [-1 0];
                disp('T070122_c1:: moving up / default condition')
            else
                % do nothing
                disp('N/A Here.')
            end
        elseif vect_sum2(1)==sum_lt && obs_lt_flag ~= 1
            if sum_lt_14_obst_flag2s == 1 || sum_lt_14_obst_flag == 1
                if (sum(obs_flag_vector2(:,1))>=1 || sum(obs_flag_vector2(:,2))>=1) && sum_dir_20_flag ~=1 && obs_lt_mid_flag ==1 
                    % calling function
                    disp('T070122_c2:: avoid_obst_dir2() executed') 
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); %%% bazinga.
                elseif sum_lt_14_obst_vect(1) == 1 && obs_up_mid_flag ~=1 % move up
                    if obs_up_mid_flag==1
                        disp('T070122_c2:: moving up-<lt/rt> / sum_lt_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);
                    else
                        moved_up_flag =1; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
                        nowPose = nowPose + [-1 0];
                        disp('T070122_c2:: moving up / sum_lt_14_obst_flag2s')
                    end
                elseif sum_lt_14_obst_vect(2) == 1  % move dn
                    if obs_dn_mid_flag ==1
                        disp('T070122_c2:: moving down-<lt/rt> / sum_lt_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);  
                    else
                        moved_up_flag =0; moved_dn_flag =1; moved_lt_flag =0; moved_rt_flag =0;
                        nowPose = nowPose + [1 0];
                        disp('T070122_c2:: moving down / sum_lt_14_obst_flag2s')
                    end
                elseif sum_lt_14_obst_vect(3) == 1 % move rt
                    if obs_rt_mid_flag == 1
                        disp('T070122_c2:: moving right-<up/dn> / sum_lt_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);  
                    else
                        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =1;
                        nowPose = nowPose + [0 1];
                        disp('T070122_c2:: moving right / sum_lt_14_obst_flag2s')
                    end
                elseif sum_lt_14_obst_flag == 1 % move left
                    %do nothing
                    if  sum_lt > 6 && obs_lt_mid_flag ~= 1 && obs_lt_mid_flag ~= 1
                        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =1; moved_rt_flag =0;
                        nowPose = nowPose + [0 -1];
                        disp('T070122_c2:: moving left / sum_lt_14_obst_flag2s')
                    elseif sum_lt > 6 && obs_lt_mid_flag == 1
                        %nowPose=avoid_obst_dir3(obs_flag_vector3,nowPose);
                        disp('T070122_c2:: testing some code <shizzle2>')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);       
                    else
                        disp('T070122_c2:: testing some code <shizzle>')
                    end
                    % uncommmented this elseif statement on 07/20/22
                elseif (obs_rt_mid_flag == 1|| obs_lt_mid_flag == 1 || obs_up_mid_flag == 1 || obs_dn_mid_flag==1) %#3KF
                    disp('T071822:: avoid_obst_dir5() executed')
                    nowPose=avoid_obst_dir5(mat_sm5_exp, obs_flag_vector3,nowPose);
                else
                    % do nothing.
                    disp('T070122_c2:: INFO: X_lt_X')
                end
                disp('moving left / sum_lt_14_obst_flag == 1')
            elseif ((sum_lt_14_olt6_flag == 1 && (sum_lt_dn_cell == 6 || sum_lt_dn_cell == 2) && sum_lt_up_cell <= 6 && bot_lt_cell_sm ~=1) || sum_sm_lt_up2_flag ==1) && sm_bot_lt_corner ~=1
                nowPose = nowPose + [1 -1];
                disp('moving lt-dn')
            elseif ((sum_lt_14_olt6_flag == 1 && (sum_lt_up_cell ==6 || sum_lt_up_cell ==2) && sum_lt_dn_cell <= 6 && bot_rt_cell_sm ~=1) || sum_sm_lt_dn2_flag ==1) && sm_top_lt_corner ~=1
                nowPose = nowPose + [-1 -1];
                disp('moving lt-up')
            elseif obs_lt_mid_flag ~=1
                moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =1; moved_rt_flag =0;
                nowPose = nowPose + [0 -1];
                disp('moving left / default condition')
            else
                % do nothing
                disp('N/A Here.')
            end
        elseif vect_sum2(1)==sum_dn && obs_dn_flag ~= 1 
            if sum_dn_14_obst_flag2s == 1 || sum_dn_14_obst_flag == 1
                if (sum(obs_flag_vector2(:,1))>=1 || sum(obs_flag_vector2(:,2))>=1) && sum_dir_20_flag ~=1 && obs_dn_mid_flag ==1
                    % calling function
                    disp('T070122_c3:: avoid_obst_dir2() executed')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); %%% bazinga.
                elseif sum_dn_14_obst_vect(1) == 1 
                    if obs_up_mid_flag == 1
                        disp('T070122_c3:: moving up-<lt/rt> / sum_dn_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);  
                    else
                        moved_up_flag =1; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
                        nowPose = nowPose + [-1 0];
                        disp('T070122_c3:: moving up / sum_dn_14_obst_flag2s')
                    end

                elseif sum_dn_14_obst_vect(2) == 1 % move lt
                    if obs_lt_mid_flag == 1
                        disp('T070122_c3:: moving left-<up/dn> / sum_dn_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); 
                    else
                        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =1; moved_rt_flag =0;
                        nowPose = nowPose + [0 -1];
                        disp('T070122_c3:: moving left / sum_dn_14_obst_flag2s')
                    end
                elseif sum_dn_14_obst_vect(3) == 1 % move rt
                    if obs_rt_mid_flag == 1
                        disp('T070122_c3:: moving right-<up/dn> / sum_dn_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);
                    else
                        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =1;
                        nowPose = nowPose + [0 1];
                        disp('T070122_c3:: moving right / sum_dn_14_obst_flag2s')
                    end
                elseif sum_dn_14_obst_flag == 1 % move dn
                    if sum_dn > 6 && obs_dn_flag ~=1 && obs_dn_mid_flag ~=1
                        moved_up_flag =0; moved_dn_flag =1; moved_lt_flag =0; moved_rt_flag =0;
                        nowPose = nowPose + [1 0];
                        disp(obs_dn_mid_flag)
                        disp('T070122_c3:: $moving down$ / sum_dn_14_obst_flag2s')
                    elseif sum_dn > 6 && obs_dn_mid_flag ==1
                        disp('T070122_c3:: testing some code <shizzle2>')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);  
                    else
                        disp('T070122_c3:: testing some code <shizzle>')
                    end
                    % uncommmented this elseif statement on 07/20/22
                elseif (obs_rt_mid_flag == 1|| obs_lt_mid_flag == 1 || obs_up_mid_flag == 1 || obs_dn_mid_flag==1) %#3KF
                    disp('T071822:: avoid_obst_dir5() executed')
                    nowPose=avoid_obst_dir5(mat_sm5_exp, obs_flag_vector3,nowPose);
                else
                    % do nothing.
                    disp('INFO: X_dn_X')
                end
            elseif ((sum_dn_14_olt6_flag == 1 && (sum_dn_lt_cell == 6 || sum_dn_lt_cell == 2) && sum_dn_rt_cell <= 6 && bot_lt_cell_sm ~=1) || sum_sm_dn_rt2_flag ==1) && sm_bot_lt_corner ~=1
                nowPose = nowPose + [1 -1];
                disp('T070122_c3:: moving dn-lt')
            elseif ((sum_dn_14_olt6_flag == 1 && (sum_dn_rt_cell == 6 || sum_dn_rt_cell == 2) && sum_dn_lt_cell <= 6 && bot_rt_cell_sm ~=1) || sum_sm_dn_lt2_flag ==1) && sm_bot_rt_corner ~=1 # quick_here_1138
                nowPose = nowPose + [1 1];
                disp('T070122_c3:: moving dn-rt')
            elseif obs_dn_mid_flag ~=1
                moved_up_flag =0; moved_dn_flag =1; moved_lt_flag =0; moved_rt_flag =0;
                nowPose = nowPose + [1 0];
               disp('T070122_c3:: moving down / default condition')
            else
                % do nothing
                disp('N/A Here.')
            end
        elseif vect_sum2(1)==sum_rt && obs_rt_flag ~= 1 
            % quick_access_point_1
            if sum_rt_14_obst_flag2s == 1 || sum_rt_14_obst_flag == 1
                if (sum(obs_flag_vector2(:,1))>=1 || sum(obs_flag_vector2(:,2))>=1) && sum_dir_20_flag ~=1 && obs_rt_mid_flag ==1 
                    % calling function
                    disp('T070122_c4:: avoid_obst_dir2() executed')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose); %%% bazinga.
                elseif sum_rt_14_obst_vect(1) == 1 % move up
                    % move obs_up_mid_flag ~=1 to be a condtion if no mid obstacle then keep traveling up; if mid obstacle then take up-lt or up-rt...
                    if obs_up_mid_flag ==1 %#check_up<lt/rt>
                        disp('T070122_c4:: moving up-<lt/rt> / sum_rt_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);
                    else
                        moved_up_flag =1; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
                        nowPose = nowPose + [-1 0];
                        disp('T070122_c4:: moving up / sum_rt_14_obst_flag2s')
                    end
                elseif sum_rt_14_obst_vect(2) == 1  % move dn
                    if obs_dn_mid_flag == 1
                        disp('T070122_c4:: moving down-<lt/rt> / sum_rt_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);   
                    else
                        moved_up_flag =0; moved_dn_flag =1; moved_lt_flag =0; moved_rt_flag =0;
                        nowPose = nowPose + [1 0];
                        disp('T070122_c4:: moving down / sum_rt_14_obst_flag2s')
                    end
                elseif sum_rt_14_obst_vect(3) == 1 % move lt
                    if obs_lt_mid_flag == 1
                        disp('T070122_c4:: moving left-<up/dn> / sum_rt_14_obst_flag2s')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);
                    else
                        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =1; moved_rt_flag =0;
                        nowPose = nowPose + [0 -1];
                        disp('T070122_c4:: moving left / sum_rt_14_obst_flag2s')
                    end
                elseif sum_rt_14_obst_flag == 1 % move rt
                    if sum_rt > 6 && obs_rt_flag ~=1 && obs_rt_mid_flag ~=1
                        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =1;
                        nowPose = nowPose + [0 1];
                        disp('T070122_c4:: moving right / sum_rt_14_obst_flag2s')
                    elseif sum_rt > 6 && obs_rt_mid_flag ==1
                        disp('T070122_c2:: testing some code <shizzle2>')
                    nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose);
                    else
                        disp('T070122_c2:: testing some code <shizzle>')
                    end
                    % uncommmented this elseif statement on 07/20/22
                elseif (obs_rt_mid_flag == 1|| obs_lt_mid_flag == 1 || obs_up_mid_flag == 1 || obs_dn_mid_flag==1) %#3KF
                    disp('T071822:: avoid_obst_dir5() executed')
                    nowPose=avoid_obst_dir5(mat_sm5_exp, obs_flag_vector3,nowPose);
                else
                    % do nothing.
                    disp('T070122_c4:: INFO: X_rt_X')
                end
                  disp('moving right / sum_rt_14_obst_flag == 1')
            elseif ((sum_rt_14_olt6_flag == 1 && (sum_rt_up_cell == 6 || sum_rt_up_cell == 2) && sum_rt_dn_cell <= 6 && top_rt_cell_sm ~=1) || sum_sm_rt_dn2_flag ==1) && sm_top_rt_corner ~=1
                nowPose = nowPose + [-1 1];
                disp('T070122_c4:: moving rt-up')
            elseif ((sum_rt_14_olt6_flag ==1 && (sum_rt_dn_cell == 6 || sum_rt_dn_cell == 2) && sum_rt_up_cell <= 6 && top_lt_cell_sm ~=1) || sum_sm_rt_up2_flag ==1) && sm_bot_rt_corner ~=1
                nowPose = nowPose + [1 1];
                disp('T070122_c4:: moving rt-dn')
            elseif obs_rt_mid_flag ~=1
                moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =1;
                nowPose = nowPose + [0 1];
                disp('T070122_c4:: moving right / default condition')
            else
                % do nothing
                disp('N/A Here.')
            end
        else
            % commented on 06/17/22 (needs to be uncommented) do nothing
            disp('INFO: ROBOT postion did not change...')
            moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
            no_dir_move_flag = 1;
            disp('no_dir_move_flag has been turned on.')
            if no_dir_move_flag == 1 && (obs_rt_mid_flag == 1|| obs_lt_mid_flag == 1 || obs_up_mid_flag == 1 || obs_dn_mid_flag==1) %#3KF
                disp('T071822:: avoid_obst_dir5() executed')
                [nowPose, no_dir_move_flag2] =avoid_obst_dir5(mat_sm5_exp, obs_flag_vector3,nowPose);
                moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
                if no_dir_move_flag2 == 1
                    avoid_multi_moves2 = zeros(1,3);
                    %lt wall
                    if obs_lt_flag == 1 && sum_up_lt_cell == 2  && obs_up_flag ~=1 && obs_up_mid_flag ~=1 
                        % move up
                        nowPose = nowPose + [-1 0]; avoid_multi_moves2(1) = 1;
                        disp('T071322_a1:: moving up')
                    elseif obs_lt_flag == 1 && sum_dn_lt_cell == 2 && obs_dn_flag ~=1 && obs_dn_mid_flag ~=1 
                        % move dn
                        nowPose = nowPose + [1 0]; avoid_multi_moves2(1) = 1;
                        disp('T071322_a4:: moving down')
                    else
                        disp('no_dir_move2_flag:: no lt wall')
                    end
                    %rt wall
                    if obs_rt_flag == 1 && sum_up_rt_cell == 2 && obs_up_flag ~=6 && obs_up_mid_flag ~=1 && sum(avoid_multi_moves2)==0
                        % move up
                        nowPose = nowPose + [-1 0]; avoid_multi_moves2(2) = 1;
                        disp('T071322_a1:: moving up')
                    elseif obs_rt_flag == 1&& sum_up_lt_cell == 2 && obs_dn_flag ~=1 && obs_dn_mid_flag ~=1 && sum(avoid_multi_moves2)==0
                        % move dn
                        nowPose = nowPose + [1 0]; avoid_multi_moves2(2) = 1;
                        disp('T071322_a4:: moving down')
                    else
                        disp('no_dir_move2_flag:: no rt wall')
                    end
                    %top wall
                    if obs_up_flag == 1 && sum_lt_up_cell == 2 && obs_lt_flag ~=1 && obs_lt_mid_flag ~=1 && sum(avoid_multi_moves2)==0
                        % move lt
                        % add condition of middle left cell 
                        nowPose = nowPose+ [0 -1]; avoid_multi_moves2(3) = 1;
                        disp('T071322_a6:: moving left')
                    elseif obs_up_flag == 1 && sum_rt_up_cell == 2 && obs_rt_flag ~=1 && obs_rt_mid_flag ~=1 && sum(avoid_multi_moves2)==0
                        % move rt
                        nowPose = nowPose + [0 1]; avoid_multi_moves2(3) = 1;
                        disp('T071322_a5:: moving right')
                    else
                        disp('no_dir_move2_flag:: no top wall')
                    end
                    %bot wall
                    if obs_dn_flag == 1 && sum_lt_dn_cell == 2 && obs_lt_flag ~=1 && obs_lt_mid_flag ~=1 && sum(avoid_multi_moves2)==0
                        nowPose = nowPose+ [0 -1]; % move lt
                        disp('T071322_a6:: moving left')
                    elseif obs_dn_flag == 1 && sum_rt_dn_cell == 2 && obs_rt_flag ~=1 && obs_rt_mid_flag ~=1 && sum(avoid_multi_moves2)==0
                        nowPose = nowPose + [0 1]; % move rt
                        disp('T071322_a5:: moving right')
                    else
                        disp('no_dir_move2_flag:: no bot wall')
                    end
                end
            end
        end
    end
    % code below only applies for the first iteration;BEGIN_TAG:KLD3
else
    if vect_sum2(1)==sum_up && obs_up_mid_flag ~= 1 && obs_up_flag ~= 1 
        moved_up_flag =1; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
        nowPose = nowPose + [-1 0];
        disp('T070522_a1 :: moving up')
    elseif vect_sum2(1)==sum_lt && obs_lt_mid_flag ~= 1 && obs_lt_flag ~= 1 
        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =1; moved_rt_flag =0;
        nowPose = nowPose+ [0 -1];
        disp('T070522_a2 :: moving left')
    elseif vect_sum2(1)==sum_dn && obs_dn_mid_flag ~= 1 && obs_dn_flag ~= 1 %
        moved_up_flag =0; moved_dn_flag =1;
        moved_lt_flag =0; moved_rt_flag =0;
        nowPose = nowPose + [1 0];
        disp('T070522_a3 :: moving down')
    elseif vect_sum2(1)==sum_rt && obs_rt_mid_flag ~= 1 && obs_rt_flag ~= 1 %
        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =1; nowPose = nowPose + [0 1];
        disp('T070522_a4 :: moving right')
    else
        % do nothing
        disp('INFO: ROBOT postion did not change...')
        moved_up_flag =0; moved_dn_flag =0; moved_lt_flag =0; moved_rt_flag =0;
    end
    %END_TAG:KLD3
end
% make vector to access all time moved_all_flags
moved_all_flags = [moved_up_flag,moved_dn_flag,moved_lt_flag,moved_rt_flag];
%------------------------------------------------------------------
end
function nowPose=avoid_obst_dir2(obs_flag_vector2,obs_flag_vector3, mat_sm5_exp, nowPose)
    sum_sm_up_lt2_flag = obs_flag_vector2(1,1); sum_sm_up_rt2_flag = obs_flag_vector2(1,2);
    sum_sm_dn_lt2_flag = obs_flag_vector2(2,1); sum_sm_dn_rt2_flag = obs_flag_vector2(2,2);
    sum_sm_lt_up2_flag = obs_flag_vector2(3,1); sum_sm_lt_dn2_flag = obs_flag_vector2(3,2);
    sum_sm_rt_up2_flag = obs_flag_vector2(4,1); sum_sm_rt_dn2_flag = obs_flag_vector2(4,2);
    dir2_excuted_flag = 1;
    if sum_sm_up_lt2_flag == 1 % move up-right
        nowPose = nowPose + [-1 1];
        disp('T062922_a1:: moving up-rt')
    elseif sum_sm_up_rt2_flag == 1 % move up-left
        nowPose = nowPose + [-1 -1];
        disp('T062922_a2:: moving up-lt')
    elseif sum_sm_dn_lt2_flag == 1 % move down-right
        nowPose = nowPose + [1 1];
        disp('T062922_a3:: moving dn-rt')
    elseif sum_sm_dn_rt2_flag == 1 % move down-left
        nowPose = nowPose + [1 -1];
        disp('T062922_a4:: moving dn-lt')
    elseif sum_sm_lt_up2_flag == 1 % move left-down
        nowPose = nowPose + [1 -1];
        disp('T062922_a5:: moving lt-dn')
    elseif sum_sm_lt_dn2_flag == 1 % move left-up
        nowPose = nowPose + [-1 -1];
        disp('T062922_a6:: moving lt-up')
    elseif sum_sm_rt_up2_flag == 1 % move right-dn
        nowPose = nowPose + [1 1];
        disp('T062922_a7::moving rt-dn')
    elseif sum_sm_rt_dn2_flag == 1 % move right-up
        nowPose = nowPose + [-1 1];
        disp('T062922_a8::moving rt-up')
    else
        disp('Nothing Executed in avoid_obst_dir2()')
        dir2_excuted_flag = 0; % no condition were satisfied above
        % do nothing
    end
    %------------------------------------------------------------------
    sum_up_lt_cell = obs_flag_vector3(1,1); sum_up_rt_cell = obs_flag_vector3(1,2);
    sum_dn_lt_cell = obs_flag_vector3(2,1); sum_dn_rt_cell = obs_flag_vector3(2,2);
    sum_lt_dn_cell = obs_flag_vector3(3,1); sum_lt_up_cell = obs_flag_vector3(3,2);
    sum_rt_dn_cell = obs_flag_vector3(4,1); sum_rt_up_cell = obs_flag_vector3(4,2);
    %#3kf
    sm_top_lt = mat_sm5_exp(2,2); sm_top_mid = mat_sm5_exp(2,3);
    sm_top_rt = mat_sm5_exp(2,4); sm_lt_mid = mat_sm5_exp(3,2);
    sm_rt_mid = mat_sm5_exp(3,4); sm_bot_lt = mat_sm5_exp(4,2);
    sm_bot_mid = mat_sm5_exp(4,3); sm_bot_rt = mat_sm5_exp(4,4);
    % execute this code if no conditions were satisfied above 
    if dir2_excuted_flag == 0
        avoid_multi_moves = zeros(1,3);
        % robot traveling up and encounters vertical obstable
        if sm_top_mid == 1 && sum_up_lt_cell == 6 && sm_top_lt == 0
            nowPose = nowPose + [-1 -1];
            disp('T070522_a1Y:: moving up-lt')
            avoid_multi_moves(1) = 1;
        elseif sm_top_mid == 1 && sum_up_rt_cell == 6 && sm_top_rt == 0
            nowPose = nowPose + [-1 1];
            avoid_multi_moves(1) = 1;
            disp('T070522_a2Y:: moving up-rt')
        else
            % do nothing.
        end
        % robot traveling right and encounters horizontal obstacle
        if sm_rt_mid == 1 && sum_rt_up_cell == 6 && sm_top_rt == 0 && sum(avoid_multi_moves)==0
            nowPose = nowPose + [-1 1];
            avoid_multi_moves(2) = 1;
            disp('T070522_a3Y:: moving rt-up')
        elseif sm_rt_mid == 1 && sum_rt_dn_cell == 6 && sm_bot_rt == 0 && sum(avoid_multi_moves)==0
            nowPose = nowPose + [1 1];
            avoid_multi_moves(2) = 1;
            disp('T070522_a4Y:: moving rt-dn')
        else
            % do nothing.
        end
        % robot traveling left and encounters horizontal obstacle
        if sm_lt_mid == 1 && sum_lt_up_cell == 6 && sm_top_lt == 0 && sum(avoid_multi_moves)==0
            nowPose = nowPose + [-1 -1];
            avoid_multi_moves(3) = 1;
            disp('T070522_a5Y:: moving lt-up')
        elseif sm_lt_mid == 1 && sum_lt_dn_cell == 6 && sm_bot_lt == 0 && sum(avoid_multi_moves)==0
            nowPose = nowPose + [1 -1];
            avoid_multi_moves(3) = 1;
            disp('T070522_a6Y:: moving lt-dn')
        else
            % do nothing.
        end
        % robot traveling down and encounters vertical obstacle
        if sm_bot_mid == 1 && sum_dn_rt_cell == 6 && sm_bot_rt == 0 && sum(avoid_multi_moves)==0
            nowPose = nowPose + [1 1];
            disp('T070522_a7Y:: moving dn-rt')
        elseif sm_bot_mid == 1 && sum_dn_lt_cell == 6 && sm_bot_lt == 0 && sum(avoid_multi_moves)==0
            nowPose = nowPose + [1 -1];
            disp('T070522_a8Y:: moving dn-lt')
        else
            % do nothing.
        end
    end
end
function [nowPose, no_dir_move_flag2]=avoid_obst_dir5(mat_sm5_exp, obs_flag_vector3,nowPose)
    no_dir_move_flag2 = 0; % important flag that will allow the robot to explore the environment more
    sum_up_lt_cell = obs_flag_vector3(1,1); sum_up_rt_cell = obs_flag_vector3(1,2);
    sum_dn_lt_cell = obs_flag_vector3(2,1); sum_dn_rt_cell = obs_flag_vector3(2,2);
    sum_lt_dn_cell = obs_flag_vector3(3,1); sum_lt_up_cell = obs_flag_vector3(3,2);
    sum_rt_dn_cell = obs_flag_vector3(4,1); sum_rt_up_cell = obs_flag_vector3(4,2);
    
    sm_top_lt = mat_sm5_exp(2,2); sm_top_mid = mat_sm5_exp(2,3);
    sm_top_rt = mat_sm5_exp(2,4); sm_lt_mid = mat_sm5_exp(3,2);
    sm_rt_mid = mat_sm5_exp(3,4); sm_bot_lt = mat_sm5_exp(4,2);
    sm_bot_mid = mat_sm5_exp(4,3); sm_bot_rt = mat_sm5_exp(4,4);
    avoid_multi_moves = zeros(1,3);
    % robot traveling up and encounters vertical obstable
    if sm_top_mid == 1 && sum_up_lt_cell == 6 && sm_top_lt == 0
        nowPose = nowPose + [-1 -1];
        avoid_multi_moves(1) = 1;
        disp('T070522_a1X:: moving up-lt')
    elseif sm_top_mid == 1 && sum_up_rt_cell == 6 && sm_top_rt == 0
        nowPose = nowPose + [-1 1];
        avoid_multi_moves(1) = 1;
        disp('T070522_a2X:: moving up-rt')
    else
        % do nothing.
    end
    % robot traveling right and encounters horizontal obstacle 
    if sm_rt_mid == 1 && sum_rt_up_cell == 6 && sm_top_rt == 0 && sum(avoid_multi_moves)==0
        nowPose = nowPose + [-1 1];
        avoid_multi_moves(2) = 1;
        disp('T070522_a3X:: moving rt-up')
    elseif sm_rt_mid == 1 && sum_rt_dn_cell == 6 && sm_bot_rt == 0 && sum(avoid_multi_moves)==0
        nowPose = nowPose + [1 1];
        avoid_multi_moves(2) = 1;
        disp('T070522_a4X:: moving rt-dn')
    else
        % do nothing.
    end
    % robot traveling left and encounters horizontal obstacle 
    if sm_lt_mid == 1 && sum_lt_up_cell == 6 && sm_top_lt == 0 && sum(avoid_multi_moves)==0
        nowPose = nowPose + [-1 -1];
        avoid_multi_moves(3) = 1;
        disp('T070522_a5X:: moving lt-up')
    elseif sm_lt_mid == 1 && sum_lt_dn_cell == 6 && sm_bot_lt == 0 && sum(avoid_multi_moves)==0
        nowPose = nowPose + [1 -1];
        avoid_multi_moves(3) = 1;
        disp('T070522_a6X:: moving lt-dn')
    else
        % do nothing.
    end
    % robot traveling down and encounters vertical obstacle
    if sm_bot_mid == 1 && sum_dn_rt_cell == 6 && sm_bot_rt == 0 && sum(avoid_multi_moves)==0
        nowPose = nowPose + [1 1];
        disp('T070522_a7X:: moving dn-rt')
    elseif sm_bot_mid == 1 && sum_dn_lt_cell == 6 && sm_bot_lt == 0 && sum(avoid_multi_moves)==0
        nowPose = nowPose + [1 -1];
        disp('T070522_a8X:: moving dn-lt')
    else
        % do nothing.
        no_dir_move_flag2 = 1;
        disp('T070522X:: nothing was executed in avoid_obst_dir5()')
    end
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
