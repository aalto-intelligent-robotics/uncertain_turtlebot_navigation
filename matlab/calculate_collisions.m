clear all;
close all;
clc;
% Set isdebug to 1 if you want to plot the trajectories at run-time. This, however,
% slows down the process severely. 
isdebug = 0; 
image = rgb2gray(imread('../maps/gt_map.png'));
% Transform the image to an occupancy map
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
% Create an occupancy map
map = robotics.OccupancyGrid(imageOccupancy,20);
methods = {'gaussian','laplace','mcdropout','slam'};
data = {'gaussian','laplace','mcdropout','slam'};
path = '../data/trajectories/';
result = cell(4,1);
for i=1:4
    collisions = {};
    file_path = strcat(path,methods{i},'/');
    num_trajectories = size(dir(file_path),1);
    for j=0:num_trajectories-1
        file_name = strcat('trajectory_', num2str(j));
        try
            file = strcat(file_path,file_name);
            trajectory = importdata(file);
        catch ME
            %disp(ME);
            disp(['Cannot open file ' strcat(file_path,file_name)])
            continue;
        end
        % Add the start-point of the robot in meters to all the
        % points along the trajectory
        trajectory(:,1) = trajectory(:,1)+30.4;
        trajectory(:,2) = trajectory(:,2)+32;
        if any(checkOccupancy(map, trajectory),'all')
            % Adds the file of the trajectory where the collision occured
            collisions{end+1} = file;
            if isdebug
                show(map);
                hold on;
                plot(trajectory(:,1),trajectory(:,2))
                pause(0.05);
            end
        end
    end
    data{i} = collisions;
    result{i} = (strcat("Collisions for method ", methods{i},": ", num2str(size(data{i},2)),"/",num2str(num_trajectories))); 
end
disp(result)
%%
% Sample a random trajectory number which lead to a collision in the slam map and plot
% the corresponding trajectory for all methods

rand_int = randi([1 size(data{4},2)],1);
trajectory_num = split(data{4}{rand_int},'_');
trajectory_num = trajectory_num{2};
plot_trajectory(map, trajectory_num, methods)
