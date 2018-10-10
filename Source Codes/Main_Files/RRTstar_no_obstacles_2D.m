%%

% implementation of the path planning algorithm RRT* without obstacles
% Johann Diep, 19 March 2016, Focusproject Scubo

% old

clf;
clear;
clc;
%%

% creates a grid

map_width=10; % map width in meter
map_height=10; % map height in meter
map_resolution=2; % cells per meter

figure(1)
map=robotics.BinaryOccupancyGrid(map_width,map_height,map_resolution); % creating map
%ij=[1 1]; % location of top right corner of obstacle, x/y-coord. in meter 
%setOccupancy(map,ij,1); % creating obstacle in map
show(map);
hold on
grid on
set(gca,'XTick', 0:1/2:10, 'YTick', 0:1/map_resolution:10) % visible pattern, x- and y-axis from 0-10m, stepsize 1/resolution

%%

% variable definition 

stepsize=0.5; % definition of the stepsize per interation
d=2; % dimension
y_RRT=10; % must be greater than (2*(1+1/d))^(1/d)*((map_width*map_height)/pi)^(1/d)=9.7721
%card=1; % only starting point in states array at the begin
%neighborhood_radius=min(y_RRT*(log(card)/card)^(1/d),stepsize); % definition of the neighborhood radius for the RRT* algorithm
neighborhood_radius=100;
x_init=0.5; % x-coord. of initial point
y_init=0.5; % y-coord. of initial point
x_final=9; % x-coord. of final point
y_final=9; % y-coord. of final point
iteration=1000; % number of sampling iteration

plot(x_init,y_init,'og','MarkerSize',10) % initialization of the start 
%plot(x_final,y_final,'og','MarkerSize',10) % initialization of the goal

states_array=[;,;,;,;]; % initialization of a states_array, x-coord./y-coord./parent (row number of states_array)/cost (edge) from start state 

% filling inital state into first row of states_array 
states_array(1,1)=x_init;
states_array(1,2)=y_init;
states_array(1,3)=1; % default for no parent
states_array(1,4)=0; % no cost at the begin

distance_array=[]; % initialization of a distance array 
neighborhood_cost_array=[]; % initialization of a cost array for the neighborhood states

%%

% calculation of the nearest state

for i=1:iteration

    card=size(states_array,1); 
    display(card/iteration*100);
    display('% of calculation completed!'); % display loading
    
    neighborhood_radius=min(y_RRT*(log(card)/card)^(1/d),stepsize); % definition of the neighborhood radius for the RRT* algorithm
    
    x_sample=map_width*rand; % random x-coord. of sample point
    y_sample=map_height*rand; % random y-coord. of sample point

    for j=1:size(states_array,1) % for each row of states_array 
    
        distance_array(j)=((x_sample-states_array(j,1))^2+(y_sample-states_array(j,2))^2)^(1/2); % distance from state to sample
    
    end

    [~,Locb]=ismember(min(distance_array),distance_array); % searching for the index of the smallest distance
    
    % nearest state
    x_near=states_array(Locb,1); 
    y_near=states_array(Locb,2);

%%   
    
    % calculation of the new state
    
    % avoiding overshoot if stepsize is larger than the distance between sampled state and nearest state 

    if stepsize < min(distance_array) % distance between sampled state and nearest state

        x_new=x_near+stepsize*cos(atan2((y_sample-y_near),(x_sample-x_near))); % x-coord. of new state
        y_new=y_near+stepsize*sin(atan2((y_sample-y_near),(x_sample-x_near))); % y-coord. of new state  
    
        edge_cost=stepsize; % cost equals stepsize
        
    else
        
        x_new=x_sample;
        y_new=y_sample;
        
        edge_cost=min(distance_array); % cost equals distance to the new state
        
    end
      
%%
    
    % filling new state into state array
    
    states_array(i+1,1)=x_new;
    states_array(i+1,2)=y_new;
    
    plot(states_array(i+1,1), states_array(i+1,2), 'or', 'MarkerSize', 3)
    
    [~,Locb]=ismember([x_near,y_near],states_array(:,1:2),'rows'); % calculating the row of the parent in the states_array
    states_array(i+1,3)=Locb; 
   
    states_array(i+1,4)=states_array(Locb, 4)+ edge_cost; % cost of parent plus cost of the edge
    
%%

    % connection along a minimum-cost path
    
    cost_default=10^5; % random high default number

    for j=1:size(states_array,1) % for each row of states_array 
            
        % searching for states within the neighborhood radius
        
        distance_to_neighbor=((x_new-states_array(j,1))^2+(y_new-states_array(j,2))^2)^(1/2); 
        
        if distance_to_neighbor < neighborhood_radius
        
            % optimize cost by searching for new parents
            
            if states_array(i+1,4) > states_array(j,4)+ distance_to_neighbor % calculation of the optimal cost to the new state
                
                %if distance_to_neighbor <= stepsize % new distance must be smaller or equal stepsize
                cost_intern=states_array(j,4)+ distance_to_neighbor;
                
                % finding the state with the shortest distance
                
                if cost_intern < cost_default
                    
                    states_array(i+1,3)=j; % setting new parent
                    states_array(i+1,4)=states_array(j,4)+ distance_to_neighbor; % setting new cost
                    %neighborhood_cost_array(j)=states_array(j,4)+distance_to_neighbor;
                    
                    cost_default=states_array(j,4)+ distance_to_neighbor; % update default cost
                    
                end
                    
                %end
                
            end
            
        end
        
    end
    
    %[~,Locb]=ismember(min(neighborhood_cost_array),neighborhood_cost_array);
    
    %states_array(i+1,3)=Locb; % setting new parent
    %states_array(i+1,4)=states_array(Locb,4)+ ((x_new-states_array(Locb,1))^2+(y_new-states_array(Locb,2))^2)^(1/2); % setting new cost 
    
%%

    % rewire the tree

    for k=1:size(states_array,1)
    
        distance_to_neighbor=((x_new-states_array(k,1))^2+(y_new-states_array(k,2))^2)^(1/2); 

        if distance_to_neighbor < neighborhood_radius
            
            % optimize cost by recalculate parent for neighbor-states
        
            if states_array(i+1,4)+distance_to_neighbor < states_array(k,4) % calculation of the optimal cost through the new state
                
                %if distance_to_neighbor <= stepsize % new distance must be smaller or equal stepsize
                
                    states_array(k,3)=i+1; % setting new parent
                    states_array(k,4)= states_array(i+1,4)+distance_to_neighbor; % setting new 
                    
                %end
                
            end
            
        end
        
    end
    
%%

    % plot the tree (inside for-loop)
    
%     plot([states_array(i,1),states_array(states_array(i,3),1)],[states_array(i,2),states_array(states_array(i,3),2)],'b')
%     drawnow;
% 
%     display('Plotting done!')
    
%%

    % termination if goal is achieved
   
%     if abs(states_array(i+1,1)-x_final)<0.2 && abs(states_array(i+1,2)-y_final)<0.2 % termination state
%         
%         break
%         
%     end     
        
end

%%

% plot the tree
% 
% for l=1:size(states_array,1)
% 
% plot([states_array(l,1),states_array(states_array(l,3),1)],[states_array(l,2),states_array(states_array(l,3),2)],'r')
% drawnow;
% 
% display(l/size(states_array,1)*100);
% display('% plotting completed');
% 
% end


%%

% plot the path

% [~,end_index]=ismember([states_array(end,1),states_array(end,2)],states_array(:,1:2),'rows'); % getting index of last row
% index=end_index;
% 
% while index~=1
% 
%     plot([states_array(index,1),states_array(states_array(index,3),1)],[states_array(index,2),states_array(states_array(index,3),2)],'b')
%     
%     index=states_array(index,3); % index update
% 
% end


