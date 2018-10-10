function [x_near, y_near, z_near, x_near_parent, y_near_parent, z_near_parent] = Nearest(states_array, x_sample, y_sample, z_sample, x_init_direction, y_init_direction, z_init_direction)
%function [x_near, y_near, x_near_parent, y_near_parent] = Nearest(states_array, x_sample, y_sample, x_init_direction, y_init_direction) % uncomment this for 2D map
%function [x_near, y_near, x_near_parent, y_near_parent, states_tree] = Nearest(states_array, x_sample, y_sample, x_init_direction, y_init_direction)

% this function finds the x- and y-coordinates of the nearest point and its parent relative to the sample point using KD-tree algorithm

%difference_array=states_array(:,1:2)-repmat([x_sample,y_sample],size(states_array,1),1); % uncomment this for 2D map

difference_array=states_array(:,1:3)-repmat([x_sample,y_sample,z_sample],size(states_array,1),1);

distance_array=sqrt(sum(difference_array.^2,2));

[~,idx]=ismember(min(distance_array),distance_array);

%states_tree=createns(states_array(:,1:2),'nsmethod','kdtree');

%[idx,~]=knnsearch(states_tree,[x_sample,y_sample],'k',1);

% nearest state
x_near=states_array(idx,1); 
y_near=states_array(idx,2);
z_near=states_array(idx,3);

% avoid dot product with zero vector
if idx~=1

    x_near_parent=states_array(states_array(idx,4),1);
    y_near_parent=states_array(states_array(idx,4),2);
    z_near_parent=states_array(states_array(idx,4),3);

else
    
    x_near_parent=x_init_direction;
    y_near_parent=y_init_direction;
    z_near_parent=z_init_direction;
    
end
    
end

