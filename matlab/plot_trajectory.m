function [] = plot_trajectory(map,trajectory_num,methods)
%PLOT_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
goal_points = importdata('../maps/goals');
goal_points(:,1) = goal_points(:,1)+30.4;
goal_points(:,2) = goal_points(:,2)+32;
path = '../data/trajectories/';

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
end

