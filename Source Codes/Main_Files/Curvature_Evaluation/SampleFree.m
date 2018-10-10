%function [x_sample, y_sample, z_sample] = SampleFree(map_width, map_height, map_length)
function [x_sample, y_sample] = SampleFree(map_width, map_height) % uncomment this for 2D map

% this function generates the x-/y- and z-coordinates of a random sample point



x_sample=map_width*rand;
%y_sample=map_width*rand;
y_sample=map_height*rand;

end

