function [ occupancy ] = getOccupancy3D(occupancy_matrix, map_resolution, check_array)

% this function returns 1, if a point hits an obstacle, otherwise it returns 0

for n=1:size(check_array,1)

    if occupancy_matrix(ceil(check_array(n,1)/map_resolution),ceil(check_array(n,2)/map_resolution),ceil(check_array(n,3)/map_resolution))==1

        occupancy=1;
        break
        
    else

        occupancy=0;

    end

end