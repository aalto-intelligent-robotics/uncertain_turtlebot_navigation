clear all;
close all;
clc;
isdebug = 0;
image = imread('../maps/slam_map.pgm');
image = rgb2gray(imread('../maps/gt_map.png'));
%%
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
%imageOccupancy(imageOccupancy>0) = 1;
map = robotics.OccupancyGrid(imageOccupancy,20);
methods = {'gaussian','laplace','mcdropout','slam'};
data = {'gaussian','laplace','mcdropout','slam'};
path = '../data/trajectories/';
num_trajectories = 400;
for i=1:4
    collisions = {};
    file_path = strcat(path,methods{i},'/');
    for j=0:num_trajectories-1
        file_name = strcat('trajectory_', num2str(j));
        try
            file = strcat(file_path,file_name);
            trajectory = importdata(file);
            trajectory(:,1) = trajectory(:,1)+30.4;
            trajectory(:,2) = trajectory(:,2)+32;
            if any(checkOccupancy(map, trajectory),'all')
                collisions{end+1} = file;
                if isdebug
                    show(map);
                    hold on;
                    plot(trajectory(:,1),trajectory(:,2))
                    pause(0.05);
                end
            end
        catch ME
            disp(ME);
            disp(['Cannot open file ' strcat(file_path,file_name)])
        end
    end
    data{i} = collisions;
end

%%
% Plot on the occupancy map
rand_int = randi([1 size(data{4},2)],1)
trajectory_num = split(data{4}{rand_int},'_');
trajectory_num = trajectory_num{2};
goal_points = importdata('../maps/goals');
goal_points(:,1) = goal_points(:,1)+30.4;
goal_points(:,2) = goal_points(:,2)+32;

show(map);
hold on;
scatter(goal_points(:,1),goal_points(:,2),'s','filled');
for i=1:4
    file_path = strcat(path,methods{i},'/');
    file_name = strcat('trajectory_', num2str(trajectory_num));
    file = strcat(file_path,file_name);
    rand_trajectory = importdata(file);
    rand_trajectory(:,1) = rand_trajectory(:,1)+30.4;
    rand_trajectory(:,2) = rand_trajectory(:,2)+32;
    plot(rand_trajectory(:,1),rand_trajectory(:,2))
end
%%
% Plot on an empty image
close all
fig1 = figure;
%imshow(image);
bg_img = ones(size(image));
imshow(bg_img)
hold on;
img_goal_points = world2grid(map,goal_points);
scatter(img_goal_points(:,2),img_goal_points(:,1),'s','filled');
saveas(fig1,'goal.png')
hold off;
 clf(fig1)
for i=1:4
imshow(bg_img)

    file_path = strcat(path,methods{i},'/');
    file_name = strcat('trajectory_', num2str(trajectory_num));
    file = strcat(file_path,file_name);
    rand_trajectory = importdata(file);
    rand_trajectory(:,1) = rand_trajectory(:,1)+30.4;
    rand_trajectory(:,2) = rand_trajectory(:,2)+32;
    img_path = world2grid(map,rand_trajectory);
    hold on;
    plot(img_path(:,2),img_path(:,1),'linewidth',1.5)
    saveas(fig1,strcat(methods{i},'.png'));
    hold off;
     clf(fig1)
end

