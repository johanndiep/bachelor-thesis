function [x_new, y_new, z_new, edge_cost] = Steer(x_sample, y_sample, z_sample, x_near, y_near, z_near, x_near_parent, y_near_parent, z_near_parent, stepsize, w_1, w_2)
%function [x_new, y_new, edge_cost] = Steer(x_sample, y_sample, x_near, y_near, x_near_parent, y_near_parent, stepsize, w_1, w_2)

% this function calculates the new state and the corresponding connection cost

%angle_between_states=acos(dot([x_sample-x_near,y_sample-y_near],[x_near-x_near_parent,y_near-y_near_parent])/(norm([x_sample-x_near,y_sample-y_near])*norm([x_near-x_near_parent,y_near-y_near_parent]))); % angle between two states, uncomment this for 2D map
angle_between_states=acos(dot([x_sample-x_near,y_sample-y_near,z_sample-z_near],[x_near-x_near_parent,y_near-y_near_parent,z_near-z_near_parent])/(norm([x_sample-x_near,y_sample-y_near,z_sample-z_near])*norm([x_near-x_near_parent,y_near-y_near_parent,z_near-z_near_parent]))); % angle between two states

% avoiding overshoot if stepsize is larger than the distance between sampled state and nearest state

%if stepsize < ((x_sample-x_near)^2+(y_sample-y_near)^2)^(1/2) % distance between sampled state and nearest state, uncomment this for 2D map
if stepsize < ((x_sample-x_near)^2+(y_sample-y_near)^2+(z_sample-z_near)^2)^(1/2) % distance between sampled state and nearest state
    
    %direction_vector=[x_sample, y_sample]-[x_near, y_near]; % uncomment this for 2D map
    
    direction_vector=[x_sample, y_sample, z_sample]-[x_near, y_near, z_near];
        
    direction_vector_scaled=direction_vector/norm(direction_vector)*stepsize;
        
    x_new=x_near+direction_vector_scaled(1);
    y_new=y_near+direction_vector_scaled(2);
    z_new=z_near+direction_vector_scaled(3);
    
    edge_cost=w_1*stepsize+w_2*angle_between_states^2; % cost equals stepsize and angle between states in square
    
else
    
    x_new=x_sample;
    y_new=y_sample;
    z_new=z_sample;
        
    edge_cost=w_1*((x_sample-x_near)^2+(y_sample-y_near)^2+(z_sample-z_near)^2)^(1/2)+w_2*angle_between_states^2; % cost equals distance and angle between states in sqaure
    
end

