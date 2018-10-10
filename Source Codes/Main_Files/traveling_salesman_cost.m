function [cost_matrix,cell_arrays] = traveling_salesman_cost(view_states, occupancy_matrix)

% this function calculates the order of the traveling salesman problem

%%

% calculating the distance between each view point via RRT*

cost_matrix(1:size(view_states,1),1:size(view_states,1))=10^5; % random high number

for i=1:size(view_states,1)

    display(i/size(view_states,1)*100); % display loading in percent
    display('% calculation completed!');  
    
    states_array_temp=RRTstar_cost_3D(view_states(i,1),view_states(i,2),view_states(i,3),view_states,occupancy_matrix); % calculate RRT*-tree to all view point.
    
    cell_arrays{i}=states_array_temp;
    
    for j=1:size(view_states,1)
               
        difference_array=states_array_temp(:,1:3)-repmat(view_states(j,:),size(states_array_temp,1),1);
        
        difference_abs=(sqrt(sum(difference_array.^2,2))); 
        
        [~,idx]=ismember(min(difference_abs),difference_abs);
        
        if states_array_temp(idx,5)<cost_matrix(i,j)
        
            % cost listed in a matrix
            
            cost_matrix(i,j)=states_array_temp(idx,5);
            cost_matrix(j,i)=states_array_temp(idx,5);
            
        end
        
        difference_abs=[]; % delete difference array
            
    end
    
    states_array_temp=[]; % delete states array
    
end

end
