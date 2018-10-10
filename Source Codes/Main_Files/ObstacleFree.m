function [ check_variable ] = ObstacleFree(x_1, y_1, z_1, x_2, y_2, z_2, map_resolution, occupancy_matrix) 
%function [ check_variable ] = ObstacleFree(x_1, y_1, x_2, y_2, map, xy) % uncomment this for 2D map

% this function check, if the connection between the sampled and the new point strikes an obstacle, in which case it returns 1, otherwise 0

%setOccupancy(map,xy,1); % creating obstacle in map, uncomment this for 2D map

%direction_vector=[x_2,y_2]-[x_1,y_1];  % uncomment this for 2D map
direction_vector=[x_2,y_2,z_2]-[x_1,y_1,z_1];

check_obstacle_resolution=0.01; % resolution of checking the line segment

check_index=1;

for m=0:check_obstacle_resolution:norm(direction_vector);% check line for obstacle in small increments

    direction_vector_scaled=direction_vector/norm(direction_vector)*m;
    
    % uncomment this for 2D map
    
    %check_array_x(check_index)=x_1+direction_vector_scaled(1);
    %check_array_y(check_index)=y_1+direction_vector_scaled(2);    
    check_array(check_index,1)=x_1+direction_vector_scaled(1);
    check_array(check_index,2)=y_1+direction_vector_scaled(2);
    check_array(check_index,3)=z_1+direction_vector_scaled(3);
    
    check_index=check_index+1;
    
end

%if ismember(1,getOccupancy(map,[check_array_x',check_array_y']))==1 % check if array consist of number 1, uncomment this for 2D grid
if ismember(1,getOccupancy3D(occupancy_matrix, map_resolution, check_array))==1 % check if array consist of number 1

    check_variable=1;
    
else
    
    check_variable=0;
    
end

% uncomment this for 2D map

 %check_array_x=[]; % delete array
 %check_array_y=[]; % delete array

check_array=[]; % delete array


