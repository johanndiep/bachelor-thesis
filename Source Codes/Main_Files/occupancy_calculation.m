function [occupancy_matrix] = occupancy_calculation(filename)

% this function discretizes a pointcloud (csv-data)
% parameter varation: map_size, cells

%%

% load data
cloud = importdata(filename);

% extract coordinates
x = cloud.data(:, 1);
y = cloud.data(:, 2);
z = cloud.data(:, 3);

% define subsample

subSampleRatio=0;
f = rand(1, length(x));

x = x(f > subSampleRatio);
y = y(f > subSampleRatio);
z = z(f > subSampleRatio);

% coordinates transformation
x=x+25;
y=y+25;
z=z+25;

% changing coordinates to rotate object
y_temp=y;
z_temp=z;
y=z_temp;
z=y_temp;

% defining map size

map_size=50;

map_length=map_size;
map_width=map_size;
map_height=map_size;

% defining the grid resolution

map_resolution=0.5;

obstacle_boundary=[map_resolution map_resolution map_resolution]; % defining obstacle length, width and height for drawing reason

occupancy_matrix(:,:,map_height/map_resolution)=zeros(map_length/map_resolution,map_width/map_resolution); % definition of a occupancy 3D matrix, setting free-space to 0 and obstacles to 1

%%

% creating occupancy matrix

for i=1:size(x,1)
    
    display(i/size(x,1)*100);
    display('% calculation completed!');
    
    if ismember(0,[x(i),y(i),z(i)]) 
        
        obstacle_states=[x(i),y(i),z(i)];
        
        n=find(~obstacle_states);
        
        obstacle_states(n)=obstacle_states(n)+0.001; % small increase
        
        occupancy_matrix(ceil(obstacle_states(1)/map_resolution),ceil(obstacle_states(2)/map_resolution),ceil(obstacle_states(3)/map_resolution))=1;
        
    else
        
        occupancy_matrix(ceil(x(i)/map_resolution),ceil(y(i)/map_resolution),ceil(z(i)/map_resolution))=1;
    
    end
    
end

[r,c,v]=ind2sub(size(occupancy_matrix),find(occupancy_matrix==1));

%%

% plotting descretization

for j=1:size(r,1)

    display(j/size(r,1)*100)
    display('% plotting completed!');
    
    setOccupancy3D(obstacle_boundary, [r(j)*map_resolution,c(j)*map_resolution,v(j)*map_resolution], 1, [0 0 0]); % edge-lengths/position/transparancy/color of obstacles, setting obstacles in 3D map

end

axis equal
grid on
