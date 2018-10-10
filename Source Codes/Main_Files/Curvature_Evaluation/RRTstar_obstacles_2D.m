%%

% implementation of the path planning algorithm RRT* with obstacles on a 2D grid
% parameter varation: stepsize, y_RRT, w_1, w_2

% Johann Diep, 6 May 2016, Focusproject Scubo 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clf;
clear;
clc;

%%

% creates a grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

map_width=10; % map width in meter
map_height=9.5; % map height in meter

map_resolution=2; % cells per meter

figure(3)

map=robotics.BinaryOccupancyGrid(map_width,map_height,map_resolution); % creating map using the Robotic System Toolbox

% edges of the grid belong to the lower left grid location, x/y-coord. in meter 

% xy=[0 2; 0.5 2; 1 2; 1.5 2; 2 2; 2.5 2; 3 2; 3.5 2; 4 2; 4.5 2; 5 2; 5.5 2; 6 2; 6.5 2; 7 2; 7.5 2; 8 2; 8.5 2; 9 2; 9.5 2; 10 2; 10.5 2; ...
%     2 4; 2.5 4; 3 4; 3.5 4; 4 4; 4.5 4; 5 4; 5.5 4; 6 4; 6.5 4; 7 4; 7.5 4; 8 4; 8.5 4; 9 4; 9.5 4; 10 4; 10.5 4; 11 4; 11.5 4; 12 4; ...
%     0 6; 0.5 6; 1 6; 1.5 6; 2 6; 2.5 6; 3 6; 3.5 6; 4 6; 4.5 6; 5 6; 5.5 6; 6 6; 6.5 6; 7 6; 7.5 6; 8 6; 8.5 6; 9 6; 9.5 6; 10 6; 10.5 6; ...
%     2 8; 2.5 8; 3 8; 3.5 8; 4 8; 4.5 8; 5 8; 5.5 8; 6 8; 6.5 8; 7 8; 7.5 8; 8 8; 8.5 8; 9 8; 9.5 8; 10 8; 10.5 8; 11 8; 11.5 8; 12 8; ...
%     0 10; 0.5 10; 1 10; 1.5 10; 2 10; 2.5 10; 3 10; 3.5 10; 4 10; 4.5 10; 5 10; 5.5 10; 6 10; 6.5 10; 7 10; 7.5 10; 8 10; 8.5 10; 9 10; 9.5 10; 10 10; 10.5 10];

%xy=[0 6; 0.5 6; 1 6; 1.5 6; 2 6; 2.5 6; 3 6; 3.5 6; 4 6; 4.5 6; 5 6; 5.5 6; 6 6; 6.5 6; 7 6; 7.5 6; 8 6; 8.5 6; 9 6; 9.5 6; 10 6; 10.5 6];

%xy=[10 9.5];

xy=[0.5 6; 1 6; 1.5 6; 2 6; 4 6; 6 6; 6.5 6; 7 6; 7.5 6; 8 6; 8.5 6; 9 6; 9.5 6; 10 6; 5.5 6; 5 6; 4.5 6; ...
    0.5 4; 1 4; 1.5 4; 2 4; 2.5 4; 3 4; 3.5 4; 4 4; 4.5 4; 5 4; 5.5 4; 6 4; 8 4; 8.5 4; 9 4; 9.5 4; 10 4; ...
    6 2; 6 2.5; 6 3; 6 3.5; ...
    4 6.5; 4 7; 4 7.5; 4 8; ...
    6.5 2; 7 2; 7.5 2; 5.5 2; 5 2; 4.5 2; 4 2; 3.5 2; 3 2; 2.5 2; 2 2; 8 2; 8.5 2; ...
    4.5 8; 5 8; 5.5 8; 3.5 8; 3 8; 2.5 8; 2 8; 6 8; 6.5 8; 7 8; 7.5 8; 8 8; 8.5 8];


setOccupancy(map,xy,1); % setting obstacle in map

show(map);
hold on
grid on
set(gca,'XTick', 0:1/map_resolution:map_width, 'YTick', 0:1/map_resolution:map_height) % visible pattern, x- and y-axis from 0-10m, stepsize 1/resolution

%%

% variable definition 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

stepsize=1; % definition of the stepsize per interation

d=2; % dimension

y_RRT=10; % must be greater than (2*(1+1/d))^(1/d)*((map_width*map_height)/pi)^(1/d)=9.7721, area over free space!

x_init=4.75; % x-coord. of initial point
y_init=2.75; % y-coord. of initial point

x_init_direction=5; % x-coord. of initial point "parent"
y_init_direction=2.75; % y-coord. of initial point "parent"

x_final=4.75; % x-coord. of final point
y_final=6.75; % y-coord. of final point

iteration=10000; % number of sampling iteration

w_1=1; % weighting factor for distance
w_2=0; % weighting factor for angle

neighborhood_radius=20; % uncomment this if neighborhood_radius should be constant, set to 0 for RRT

% plotting initial and final state in map 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot(x_init,y_init,'og','MarkerSize',10) % initialization of the start 
plot(x_final,y_final,'og','MarkerSize',20) % initialization of the goal

% filling inital state into first row of states_array 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

states_array=[;,;,;,;]; % initialization of a states_array, x-coord./y-coord./parent (row number of states_array)/cost from start state 

states_array(1,1)=x_init;
states_array(1,2)=y_init;
states_array(1,3)=1; % default for no parent
states_array(1,4)=0; % no cost at the begin

% initialization of some other arrays and variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

distance_array=[]; % initialization of a distance array 

neighbors=[]; % initialization of a neighbors array

index=-1; % variable for plotting

%%

% RRT* algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

counter=2; % to avoid 0-states in the for-loop

for i=1:iteration
    
    card=size(states_array,1); % needed for the neighborhood-radius

    display(i/iteration*100); % display loading in percent
    display('% calculation completed!');
    
    neighborhood_radius=min(y_RRT*(log(card)/card)^(1/d),stepsize); % definition of the neighborhood radius for the RRT* algorithm, uncomment this to use a constant radius
    
    [x_sample,y_sample]=SampleFree(map_width, map_height); % generating a random sample point
    
    [x_near,y_near,x_near_parent,y_near_parent]=Nearest(states_array, x_sample, y_sample, x_init_direction, y_init_direction); % finding the nearest point in the tree relative to the random sample point by using KD-tree algorithm

    %[x_near,y_near,x_near_parent,y_near_parent, states_tree]=Nearest(states_array, x_sample, y_sample, x_init_direction, y_init_direction); % finding the nearest point in the tree relative to the random sample point by using KD-tree algorithm
    
    [x_new, y_new, edge_cost]=Steer(x_sample, y_sample, x_near, y_near, x_near_parent, y_near_parent, stepsize,w_1,w_2); % calculates the new state and the corresponding edge cost
    
    check_variable=ObstacleFree(x_near, y_near, x_new, y_new, map, xy); % check connection in small increments
    
    if check_variable==0 % connection does not touch any obstacles
        
        % filling new state into state array
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
        states_array(counter,1)=x_new;
        states_array(counter,2)=y_new;
            
        [~,Locb]=ismember([x_near,y_near],states_array(:,1:2),'rows'); % calculating the row of the parent in the states_array
        states_array(counter,3)=Locb; 
   
        states_array(counter,4)=states_array(Locb, 4)+ edge_cost; % cost of parent plus cost of the connection
        
        %states_tree=createns(states_array(:,1:2),'nsmethod','kdtree'); % update states tree with new state, this step is not needed
        
        % connect along a minimum-cost path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        cost_default=10^10; % random high default number
                    
        % searching for states within the neighborhood radius using KD-tree algorithm
        
        distance_array=states_array(:,1:2)-repmat([states_array(counter,1),states_array(counter,2)],size(states_array,1),1);

        distance_from_neighbor=sqrt(sum(distance_array.^2,2));

        neighbors=find(distance_from_neighbor<=neighborhood_radius);
        
        distance_from_neighbor=distance_from_neighbor(neighbors); % update distane array with only neighboring states
                
        %[neighbors,distance_from_neighbor]=rangesearch(states_tree,[states_array(counter,1),states_array(counter,2)],neighborhood_radius); % finding the neighbors within the neighborhood radius and the corresponding distance
            
        %neighbors=cell2mat(neighbors); % convert cell into matrix
                
        %distance_from_neighbor=cell2mat(distance_from_neighbor); % convert cell into matrix
                
        % optimize cost by searching for new parent
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        for j=1:size(neighbors,1); % change to 2 if working with kd-tree
            
            % avoid dot product with zero vector
            if neighbors(j)~=1
            
                angle_from_neighbor=acos(dot([states_array(counter,1)-states_array(neighbors(j),1),states_array(counter,2)-states_array(neighbors(j),2)],[states_array(neighbors(j),1)-states_array(states_array(neighbors(j),3),1),states_array(neighbors(j),2)-states_array(states_array(neighbors(j),3),2)])/(norm([states_array(counter,1)-states_array(neighbors(j),1),states_array(counter,2)-states_array(neighbors(j),2)])*norm([states_array(neighbors(j),1)-states_array(states_array(neighbors(j),3),1),states_array(neighbors(j),2)-states_array(states_array(neighbors(j),3),2)])));
            
            else    
                
                angle_from_neighbor=acos(dot([states_array(counter,1)-states_array(neighbors(j),1),states_array(counter,2)-states_array(neighbors(j),2)],[states_array(neighbors(j),1)-x_init_direction,states_array(neighbors(j),2)-y_init_direction])/(norm([states_array(counter,1)-states_array(neighbors(j),1),states_array(counter,2)-states_array(neighbors(j),2)])*norm([states_array(neighbors(j),1)-x_init_direction,states_array(neighbors(j),2)-y_init_direction])));
               
            end                          

            if states_array(counter,4) > states_array(neighbors(j),4)+ w_1*distance_from_neighbor(j)+w_2*angle_from_neighbor^2 % calculation of the optimal cost to the new state by comparing the current state cost with the cost through each neighbor
 
                cost_intern=states_array(neighbors(j),4)+ w_1*distance_from_neighbor(j)+w_2*angle_from_neighbor^2; % save cost through neighbor into a intermediate variable for comparison reason

                if cost_intern<cost_default

                    if ObstacleFree(states_array(counter,1),states_array(counter,2),states_array(neighbors(j),1),states_array(neighbors(j),2),map, xy)==0 % check connection in small increments

                        states_array(counter,3)=neighbors(j); % setting new parent
                        states_array(counter,4)=states_array(neighbors(j),4)+ w_1*distance_from_neighbor(j)+w_2*angle_from_neighbor^2; % setting new cost

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
            if states_array(counter,3)~=1
            
                angle_to_neighbor=acos(dot([states_array(neighbors(k),1)-states_array(counter,1),states_array(neighbors(k),2)-states_array(counter,2)],[states_array(counter,1)-states_array(states_array(counter,3),1),states_array(counter,2)-states_array(states_array(counter,3),2)])/(norm([states_array(neighbors(k),1)-states_array(counter,1),states_array(neighbors(k),2)-states_array(counter,2)])*norm([states_array(counter,1)-states_array(states_array(counter,3),1),states_array(counter,2)-states_array(states_array(counter,3),2)])));

            else
         
                angle_to_neighbor=acos(dot([states_array(neighbors(k),1)-states_array(counter,1),states_array(neighbors(k),2)-states_array(counter,2)],[states_array(counter,1)-x_init_direction,states_array(counter,2)-y_init_direction])/(norm([states_array(neighbors(k),1)-states_array(counter,1),states_array(neighbors(k),2)-states_array(counter,2)])*norm([states_array(counter,1)-x_init_direction,states_array(counter,2)-y_init_direction])));
                
            end
                
            distance_to_neighbor=distance_from_neighbor; % reusing the generated distance array
        
            % optimize cost by searching for new parents
            
            if states_array(counter,4)+w_1*distance_to_neighbor(k)+w_2*angle_to_neighbor^2 < states_array(neighbors(k),4) % calculation of the optimal cost through the new state     
                                    
                if ObstacleFree(states_array(counter,1),states_array(counter,2),states_array(neighbors(k),1),states_array(neighbors(k),2),map, xy)==0
                    
                    states_array(neighbors(k),3)=counter; % setting new parent
                    states_array(neighbors(k),4)=states_array(counter,4)+w_1*distance_to_neighbor(k)+w_2*angle_to_neighbor^2; % setting new cost
                    
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
    
end

%%

% plot the tree
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

startingpoints = states_array(:,1:2); % starting points of connections
endpoints = states_array(states_array(:,3),1:2); % end points of connections

plot([startingpoints(:,1) endpoints(:,1)]', [startingpoints(:,2) endpoints(:,2)]','r') % plot connections
plot(startingpoints(:,1), startingpoints(:,2), 'or', 'MarkerSize', 3) % plot states

%%

% plot the path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cost_default=10^10;

for m=1:size(states_array,1)
    
    display('finding the path');
    
    if abs(states_array(m,1)-x_final)<0.2 && abs(states_array(m,2)-y_final)<0.2 % threshold for final state
        
        if states_array(m,4)<cost_default
        
            [~,Locb]=ismember([states_array(m,1),states_array(m,2)],states_array(:,1:2),'rows'); % searching for the index of the final state
            index=Locb;
            
            cost_default=states_array(m,4); % update default cost
            
        end
        
    end
    
end

while index~=1

    plot([states_array(index,1),states_array(states_array(index,3),1)],[states_array(index,2),states_array(states_array(index,3),2)],'b','LineWidth',2) % plot backwards
    drawnow;
    index=states_array(index,3); % index update

end

display('path found');