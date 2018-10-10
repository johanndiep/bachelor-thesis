function [states_array] = RRTstar_cost_3D(x_init,y_init,z_init, final_states, occupancy_matrix)

% this function calculates the states via RRT* (euclidean distance)

%%

% initialization

map_width=50; % map width in meter
map_height=50; % map height in meter
map_length=50; % map length in meter (for 3D)

map_resolution=0.5; % meter per cell

stepsize=1; % definition of the stepsize per interation

d=3; % dimension 
y_RRT=10; % must be greater than (2*(1+1/d))^(1/d)*((map_width*map_height*map_length)/(4/3*pi))^(1/d)=8.4567 for 3D, area over free space!

% not needed for this function

x_init_direction=0; % x-coord. of initial point "parent"
y_init_direction=0; % y-coord. of initial point "parent"
z_init_direction=0; % z-coord. of initial point  "parent"

%iteration=5000; % number of sampling iteration
iteration=1;
iteration_number=1;

w_1=0; % weighting factor for distance
w_2=1; % weighting factor for angle, not needed for this function

neighborhood_radius=90; % uncomment this if neighborhood_radius should be constant, set to 0 for RRT

states_array=[;,;,;,;,;]; % initialization of a states_array, x-coord./y-coord./z-coord./parent (row number of states_array)/cost from start state 

states_array(1,1)=x_init;
states_array(1,2)=y_init;
states_array(1,3)=z_init;
states_array(1,4)=1; % default for no parent
states_array(1,5)=0; % no cost at the begin

distance_array=[]; % initialization of a distance array 

neighbors=[]; % initialization of a neighbors array

index=-1; % variable for plotting

%%

% RRT* algorithm

counter=2; % to avoid 0-states in the for-loop

while iteration
    
    card=size(states_array,1); % needed for the neighborhood-radius

    display(iteration_number); % display iteration number
    display('% Finding all the paths.');
    
    neighborhood_radius=min(y_RRT*(log(card)/card)^(1/d),stepsize); % definition of the neighborhood radius for the RRT* algorithm, uncomment this to use a constant radius
    
    [x_sample,y_sample,z_sample]=SampleFree_Scanning(final_states, map_width, map_height, map_length); % generating a random sample point around the final states or in the whole map 
    
    [x_near,y_near,z_near,x_near_parent,y_near_parent,z_near_parent]=Nearest(states_array, x_sample, y_sample, z_sample, x_init_direction, y_init_direction, z_init_direction); % finding the nearest point in the tree relative to the random sample point by using matrix multiplication
    
    [x_new, y_new, z_new, edge_cost]=Steer(x_sample, y_sample, z_sample, x_near, y_near, z_near, x_near_parent, y_near_parent, z_near_parent, stepsize, w_1, w_2); % calculates the new state and the corresponding edge cost
            
    check_variable=ObstacleFree(x_near, y_near, z_near, x_new, y_new, z_new, map_resolution, occupancy_matrix); % check connection in small increments
    
    if check_variable==0 % connection does not touch any obstacles
        
        % filling new state into state array
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
        states_array(counter,1)=x_new;
        states_array(counter,2)=y_new;
        states_array(counter,3)=z_new;
            
        [~,Locb]=ismember([x_near,y_near,z_near],states_array(:,1:3),'rows'); % calculating the row of the parent in the states_array
        states_array(counter,4)=Locb; 
   
        states_array(counter,5)=states_array(Locb, 5)+ edge_cost; % cost of parent plus cost of the connection
                
        % connect along a minimum-cost path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        cost_default=10^10; % random high default number
                    
        % searching for states within the neighborhood radius using KD-tree algorithm
        
        distance_array=states_array(:,1:3)-repmat([states_array(counter,1),states_array(counter,2),states_array(counter,3)],size(states_array,1),1);
        
        distance_from_neighbor=sqrt(sum(distance_array.^2,2));
        
        neighbors=find(distance_from_neighbor<=neighborhood_radius);
        
        distance_from_neighbor=distance_from_neighbor(neighbors); % update distane array with only neighboring states
                
        % optimize cost by searching for new parent
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        for j=1:size(neighbors,1); % change to 2 if working with kd-tree
            
            % avoid dot product with zero vector
            
            if neighbors(j)~=1
            
                angle_from_neighbor=acos(dot([states_array(counter,1)-states_array(neighbors(j),1),states_array(counter,2)-states_array(neighbors(j),2),states_array(counter,3)-states_array(neighbors(j),3)],[states_array(neighbors(j),1)-states_array(states_array(neighbors(j),4),1),states_array(neighbors(j),2)-states_array(states_array(neighbors(j),4),2),states_array(neighbors(j),3)-states_array(states_array(neighbors(j),4),3)])/(norm([states_array(counter,1)-states_array(neighbors(j),1),states_array(counter,2)-states_array(neighbors(j),2),states_array(counter,3)-states_array(neighbors(j),3)])*norm([states_array(neighbors(j),1)-states_array(states_array(neighbors(j),4),1),states_array(neighbors(j),2)-states_array(states_array(neighbors(j),4),2),states_array(neighbors(j),3)-states_array(states_array(neighbors(j),4),3)])));
            
            else    
                
                angle_from_neighbor=acos(dot([states_array(counter,1)-states_array(neighbors(j),1),states_array(counter,2)-states_array(neighbors(j),2),states_array(counter,3)-states_array(neighbors(j),3)],[states_array(neighbors(j),1)-x_init_direction,states_array(neighbors(j),2)-y_init_direction,states_array(neighbors(j),3)-z_init_direction])/(norm([states_array(counter,1)-states_array(neighbors(j),1),states_array(counter,2)-states_array(neighbors(j),2),states_array(counter,3)-states_array(neighbors(j),3)])*norm([states_array(neighbors(j),1)-x_init_direction,states_array(neighbors(j),2)-y_init_direction,states_array(neighbors(j),3)-z_init_direction])));
               
            end                          

            if states_array(counter,5) > states_array(neighbors(j),5)+ w_1*distance_from_neighbor(j)+w_2*angle_from_neighbor^2 % calculation of the optimal cost to the new state by comparing the current state cost with the cost through each neighbor
            
                cost_intern=states_array(neighbors(j),5)+ w_1*distance_from_neighbor(j)+w_2*angle_from_neighbor^2; % save cost through neighbor into a intermediate variable for comparison reason
                
                if cost_intern<cost_default

                    if ObstacleFree(states_array(neighbors(j),1),states_array(neighbors(j),2),states_array(neighbors(j),3),states_array(counter,1),states_array(counter,2),states_array(counter,3), map_resolution, occupancy_matrix)==0 % check connection in small increments
                    
                        states_array(counter,4)=neighbors(j); % setting new parent

                        states_array(counter,5)=states_array(neighbors(j),5)+w_1*distance_from_neighbor(j)+w_2*angle_from_neighbor^2; % setting new cost
                        
                        cost_default=cost_intern; % update default cost

                    end

                end

            end
                
        end
        
        % rewire the tree
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % searching for states within the neighborhood radius by using the generated neighbors and distance array generated previously

        for k=1:size(neighbors,1) % for each row of states_array, change to 2 if working with kd-tree
         
            % avoid dot product with zero vector
            
            if states_array(counter,4)~=1
                
                angle_to_neighbor=acos(dot([states_array(neighbors(k),1)-states_array(counter,1),states_array(neighbors(k),2)-states_array(counter,2),states_array(neighbors(k),3)-states_array(counter,3)],[states_array(counter,1)-states_array(states_array(counter,4),1),states_array(counter,2)-states_array(states_array(counter,4),2),states_array(counter,3)-states_array(states_array(counter,4),3)])/(norm([states_array(neighbors(k),1)-states_array(counter,1),states_array(neighbors(k),2)-states_array(counter,2),states_array(neighbors(k),3)-states_array(counter,3)])*norm([states_array(counter,1)-states_array(states_array(counter,4),1),states_array(counter,2)-states_array(states_array(counter,4),2),states_array(counter,3)-states_array(states_array(counter,4),3)])));
                
            else
         
                angle_to_neighbor=acos(dot([states_array(neighbors(k),1)-states_array(counter,1),states_array(neighbors(k),2)-states_array(counter,2),states_array(neighbors(k),3)-states_array(counter,3)],[states_array(counter,1)-x_init_direction,states_array(counter,2)-y_init_direction,states_array(counter,3)-z_init_direction])/(norm([states_array(neighbors(k),1)-states_array(counter,1),states_array(neighbors(k),2)-states_array(counter,2),states_array(neighbors(k),3)-states_array(counter,3)])*norm([states_array(counter,1)-x_init_direction,states_array(counter,2)-y_init_direction,states_array(counter,3)-z_init_direction])));
                
            end
                
            distance_to_neighbor=distance_from_neighbor; % reusing the generated distance array
        
            % optimize cost by searching for new parents
            
            if states_array(counter,5)+w_1*distance_to_neighbor(k)+w_2*angle_to_neighbor^2 < states_array(neighbors(k),5) % calculation of the optimal cost through the new state
            
                if ObstacleFree(states_array(counter,1),states_array(counter,2),states_array(counter,3),states_array(neighbors(k),1),states_array(neighbors(k),2),states_array(neighbors(k),3), map_resolution, occupancy_matrix)==0
    
                    states_array(neighbors(k),4)=counter; % setting new parent
                    
                    states_array(neighbors(k),5)=states_array(counter,5)+w_1*distance_to_neighbor(k)+w_2*angle_to_neighbor^2;
                    
                end
                        
            end
    
        end         
        
    % update variables and deleting arrays     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
    counter=counter+1; % increment counter variable for next loop   

    neighbors=[]; % delete neighbors array
    distance_from_neighbor=[]; % delete first distance array
    distance_to_neighbor=[]; % delete second distance array
    
    end

    
    % check if there are paths to final states    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    for p=1:size(final_states,1)

        search_coordinates=states_array(:,1:3)-repmat(final_states(p,:),size(states_array,1),1);

        search_distances=(sqrt(sum(search_coordinates.^2,2))); % calculate distance and list them in a vector

        if ismember(1,search_distances<0.2)
            
            search_array(p)=1;
            
        else
            
            search_array(p)=0;
            
        end
            
    end
    
    if ismember(0,search_array)~=1 
        
        break;
        
    end
    
    search_array=[]; % delete search array for next iteration
    
    iteration_number=iteration_number+1;
    
end

end