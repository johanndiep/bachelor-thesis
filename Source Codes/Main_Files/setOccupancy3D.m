function [ ] = setOccupancy3D(edges, origin, alpha, color) 

% this function set defined obstacle in the map

origin=origin-edges;

XYZ = {[0 0 0 0]  [0 0 1 1]  [0 1 1 0] ; ...
       [1 1 1 1]  [0 0 1 1]  [0 1 1 0] ; ...
       [0 1 1 0]  [0 0 0 0]  [0 0 1 1] ; ...
       [0 1 1 0]  [1 1 1 1]  [0 0 1 1] ; ...
       [0 1 1 0]  [0 0 1 1]  [0 0 0 0] ; ...
       [0 1 1 0]  [0 0 1 1]  [1 1 1 1]   ...
      };

XYZ = mat2cell(cellfun( @(x,y,z) x*y+z, ...
               XYZ, ...
               repmat(mat2cell(edges,1,[1 1 1]),6,1), ...
               repmat(mat2cell(origin,1,[1 1 1]),6,1), ...
               'UniformOutput',false), ...
               6, ...
               [1 1 1]);


cellfun(@patch,XYZ{1},XYZ{2},XYZ{3},...
        repmat({color},6,1),...
        repmat({'FaceAlpha'},6,1),...
        repmat({alpha},6,1));

end