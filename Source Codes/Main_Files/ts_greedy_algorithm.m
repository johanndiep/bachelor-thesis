function [order] = ts_greedy_algorithm(cost_matrix)

% this function calculates the order of the states by using the greedy
% algorithm

states_number=size(cost_matrix,1);

states_index_array=2:states_number; % from index 2 since index 1 is starting point 

current_state_index=1;

order_index=1;
order(order_index)=current_state_index;

% setting diagonal element of cost matrix to a random high number

for i=1:size(cost_matrix,1)
    
    cost_matrix(i,i)=10^5;
    
end


while isempty(states_index_array)~=1
    
    order_index=order_index+1;
    
    [~,new_state_index]=ismember(min(cost_matrix(:,current_state_index)),cost_matrix(:,current_state_index));
    
    cost_matrix(:,current_state_index)=10^5; % setting column to a random high number
    cost_matrix(current_state_index,:)=10^5; % setting row to a random high number
    
    current_state_index=new_state_index;
    
    [~,current_state_index_temp]=ismember(current_state_index,states_index_array);
    
    states_index_array(current_state_index_temp)=[];
    
    order(order_index)=current_state_index;
        
end

end