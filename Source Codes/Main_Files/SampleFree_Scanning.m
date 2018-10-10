function [x_sample, y_sample, z_sample] = SampleFree_Scanning(final_states, map_width, map_height, map_length)

% this function generates the x-/y- and z-coordinates of a random sample
% point around the final states

check_index=randi([1,2]);

if check_index==1 

    x_sample=map_length*rand;
    y_sample=map_width*rand;
    z_sample=map_height*rand;
    
else

    index=randi([1,size(final_states,1)]);
    
    check_distance=2; % scope around state coordinate
    
    x_sample=check_distance*rand+(final_states(index,1)-check_distance/2);
    y_sample=check_distance*rand+(final_states(index,2)-check_distance/2);
    z_sample=check_distance*rand+(final_states(index,3)-check_distance/2);

end

end