function [view_states, occupancy_matrix_extended] = view_states_calculation(occupancy_matrix, x_init, y_init, z_init)

% this function calculates the best view states to scan the desired object,
% first state is the starting point

%%

% defining variables

%angleview_horizontal=70; % horizontal angle of view of the front camera
angleview_vertical=40; % vertical angle of view of the front camera

map_resolution=0.5;

view_states=[;,;,;];

%%

% extending the occupancy_matrix to a box for plotting reason

occupancy_matrix_extended=occupancy_matrix;

[x,y,z]=ind2sub(size(occupancy_matrix),find(occupancy_matrix==1));

for i=min(x):max(x)
    
    for j=min(y):max(y)
        
        for k=min(z):max(z)
            
            if occupancy_matrix_extended(i,j,k)~=1 
            
                occupancy_matrix_extended(i,j,k)=1;
                
            end
            
        end
        
    end
    
end

%%

occupancy_height=(max(z)-min(z))*map_resolution;

view_states_distance=(occupancy_height/2)/tan(angleview_vertical/360*2*pi);

% view states in x-direction

view_states(1,1)=x_init;
view_states(1,2)=y_init;
view_states(1,3)=z_init;

view_states(2,1)=min(x)*map_resolution-view_states_distance;
view_states(2,2)=(max(y)*map_resolution-min(y)*map_resolution)/2+min(y)*map_resolution;
view_states(2,3)=(max(z)*map_resolution-min(z)*map_resolution)/2+min(z)*map_resolution;

view_states(3,1)=max(x)*map_resolution+view_states_distance;
view_states(3,2)=(max(y)*map_resolution-min(y)*map_resolution)/2+min(y)*map_resolution;
view_states(3,3)=(max(z)*map_resolution-min(z)*map_resolution)/2+min(z)*map_resolution;

% view states in y-direction

view_states(4,1)=(max(x)*map_resolution-min(x)*map_resolution)/2+min(x)*map_resolution;
view_states(4,2)=min(y)*map_resolution-view_states_distance;
view_states(4,3)=(max(z)*map_resolution-min(z)*map_resolution)/2+min(z)*map_resolution;

view_states(5,1)=(max(x)*map_resolution-min(x)*map_resolution)/2+min(x)*map_resolution;
view_states(5,2)=max(y)*map_resolution+view_states_distance;
view_states(5,3)=(max(z)*map_resolution-min(z)*map_resolution)/2+min(z)*map_resolution;

% view states in z-direction

% view_states(5,1)=(max(x)*0.5-min(x)*0.5)/2;
% view_states(5,2)=(max(y)*0.5-min(y)*0.5)/2;
% view_states(5,3)=min(z)*0.5-view_states_distance;

view_states(6,1)=(max(x)*map_resolution-min(x)*map_resolution)/2+min(x)*map_resolution;
view_states(6,2)=(max(y)*map_resolution-min(y)*map_resolution)/2+min(y)*map_resolution;
view_states(6,3)=max(z)*map_resolution+view_states_distance;

plot3(view_states(:,1), view_states(:,2), view_states(:,3), 'ob', 'MarkerSize', 3) % plot states


end
