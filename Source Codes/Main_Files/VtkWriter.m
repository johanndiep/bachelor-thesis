classdef VtkWriter
    %VTKWRITER  Simple function to write RRT* results to VTK files
    %   No need to be a class, but way more convenient to write and test
    
    properties(Constant, Access=private)
        valid_types = {'Int8',  'Int16',  'Int32',  'Int64', 'UInt8', ...
            'UInt16', 'UInt32', 'UInt64', 'Float32', 'Float64', 'String'};
    end
    
    methods(Static)
        
        function [ fnameout ] = validate_filename(fname, ending)
            % VALIDATE_FILENAME Helper function to validate a filename
            % (basically checks that the filename is not empty and that it
            % has the desired ending
            
            ending_length = length(ending);
            fname_length = length(fname);
            if isempty(fname)
                error('filename must not be empty')
            elseif fname_length <= ending_length
                fnameout = [fname, ending];
            elseif strcmp(fname(fname_length-ending_length+1:end), ending)
                fnameout = fname;
            else
                fnameout = [fname, ending];
            end
            
        end
        
        function [ doc ] = make_vtk_xml(type)
            % MAKE_VTK_XML Helper function to create a vtk xml file
            % Simply creates the top xml node and fills the standard 
            % attributes
            % Arguments:
            %   - type: The type of file which is created (e.g. PolyData)
          
            doc = com.mathworks.xml.XMLUtils.createDocument('VTKFile');
            docroot = doc.getDocumentElement;
            docroot.setAttribute('version', '1.0');
            docroot.setAttribute('byte_order', 'LittleEndian');
            docroot.setAttribute('header_type', 'UInt64');
            docroot.setAttribute('type', type);

        end
        
        function [ node ] = make_dataarray(doc, values, name, type)
            % MAKE_DATAARRAY Helper function to create a dataarray xml node
            % These nodes are often used in vtk xml files.
            % Arguments:
            %   - doc: the xml doc instance from which the new node will be
            %          created
            %   - values: The values which will be filled into the array.
            %             The array is expected to have dimensions n*d. So
            %             be careful with 1*n arrays!
            %   - name: A string with a name
            %   - type: A valid vtk type
    
            % check the provided type
            if ~ismember(type, VtkWriter.valid_types);
                error('type should be one of: %s', ...
                    strjoin(VtkWriter.valid_types, ', '));
            end

            % get the number of components
            nrc = size(values, 2);

            % get range min and max
            rangemin = min(values(:));
            rangemax = max(values(:));
            
            % write the xml node
            node = doc.createElement('DataArray');
            
            % set all the attributes
            node.setAttribute('type', type);
            node.setAttribute('Name', name);
            node.setAttribute('NumberOfComponents', num2str(nrc));
            node.setAttribute('RangeMin', num2str(rangemin));
            node.setAttribute('RangeMax', num2str(rangemax));
            
            % set the text content (values)
            values_str = strtrim(sprintf(' %g', values'));
            node.setTextContent(values_str);
  
        end
        
        function [ ] = write_rrt_path(states_array, goal, fname, ...
                reach_radius, bestn)
            % WRITE_RRT_PATH Writes the best n paths to a vtk file
            % Arguments:
            %   - states_array: The states of the tree. expected format is
            %               x,y,z,parend_id,cost per row. --> n*5
            %   - goal:     The goal point (e.g. [1 1 1])
            %   - fname:    the filename. if the filename does not end in
            %               .vtp the ending is added automatically.
            %   - reach_radius: the maximal distance of the path end point
            %               to the goal
            %   - bestn (optional): The number of paths which should be
            %               written. If ommitted only the best path will be
            %               written.
            
            % set default value for bestn if not provided
            if nargin < 5
                bestn = 1;
            end
            
            % get all the states that are close enough
            nrstates = size(states_array, 1);
            distances = sqrt(...
                          sum(...
                            (repmat(goal, [nrstates 1]) - ...
                               states_array(:,1:3)).^2, 2));
            candidates = distances <= reach_radius;
            candidates_idx = find(candidates);
            
            % limit bestn to the number of candidates
            bestn = min(bestn, length(candidates_idx));
            
            % get the sorted indices (according to cost) of the candidates
            % TODO: should we always add the final cost? What if we have
            % used angles?
            [costs, sort_index] = sort(states_array(candidates, 5) ...
                + distances(candidates));
            
            % limit to the bestn
            costs = costs(1:bestn);
            sort_index = sort_index(1:bestn);
            
            % get the corresponding indices on the full states_array
            indices = candidates_idx(sort_index);
            
            % build up the points, linesconn and linesoff arrays
            points = [states_array(1,1:3); goal];
            linesconn = [];
            linesoff = [];
            for index = indices'
                % for each index in indices we need to trace back the path
                % to the root state
                path_id = index;
                parent_id = states_array(index, 4);
                while parent_id ~= 1
                   path_id(end+1) = parent_id;
                   parent_id = states_array(parent_id, 4);
                end
                % append the new points
                p_before = size(points, 1);
                points = [points; states_array(path_id, 1:3)];
                p_after = size(points, 1);
                % update linesconn and linesoff
                linesconn = [linesconn, 1, (p_before):(p_after-1), 0];
                % ohne end punkt
                %linesconn = [linesconn, (p_before):(p_after-1), 0];

                linesoff(end+1) = length(linesconn);
            end
            
            % helper variables
            numpoints = length(points);
            numlines = length(costs);
            
            
            % start a new xml doc
            doc = VtkWriter.make_vtk_xml('PolyData');
            n_docNode = doc.getDocumentElement;
            n_polydata = doc.createElement('PolyData');
            n_docNode.appendChild(n_polydata);
            
            n_piece = doc.createElement('Piece');
            n_piece.setAttribute('NumberOfPoints', num2str(numpoints));
            n_piece.setAttribute('NumberOfVerts', '0');
            n_piece.setAttribute('NumberOfLines', num2str(numlines));
            n_piece.setAttribute('NumberOfStrips', '0');
            n_piece.setAttribute('NumberOfPolys', '0');
            n_polydata.appendChild(n_piece);
            
            n_celldata = doc.createElement('CellData');
            n_celldataarray = VtkWriter.make_dataarray(doc, ...
                costs, 'cost', 'Float32');
            n_celldata.appendChild(n_celldataarray);
            n_celldataarray = VtkWriter.make_dataarray(doc, ...
                (1:numlines)', 'score', 'Float32');
            n_celldata.appendChild(n_celldataarray);
            n_piece.appendChild(n_celldata);
            
            n_points = doc.createElement('Points');
            n_pointsarray = VtkWriter.make_dataarray(doc, ...
                points, 'Points', 'Float32');
            n_points.appendChild(n_pointsarray);
            n_piece.appendChild(n_points);
            
            n_lines = doc.createElement('Lines');
            n_linesconnectivity = VtkWriter.make_dataarray(doc, ...
                linesconn', 'connectivity', 'Int32');
            n_linesoffsets = VtkWriter.make_dataarray(doc, ...
                linesoff', 'offsets', 'Int32');
            n_lines.appendChild(n_linesconnectivity);
            n_lines.appendChild(n_linesoffsets);
            n_piece.appendChild(n_lines);
            
            fname_out = VtkWriter.validate_filename(fname, '.vtp');
            xmlwrite(fname_out, doc);
            
        end

        
        function [ ] = write_rrt(states_array, fname)
            % WRITE_RRT Function to write an RRT to vtk.
            % Arguments:
            %   - states_array: The states of the tree. expected format is
            %           x,y,z,parend_id,cost per row. --> n*5
            %   - fname: the filename. if the filename does not end in .vtp
            %           the ending is added automatically.
            
            
            % the number of points
            numpoints = size(states_array, 1);
            
            % line connectivity array. each entry contains the starting and
            % the ending index of a line
            linesconn = [1:(numpoints-1); states_array(2:end,4)'-1];
            linesconn = linesconn(:);
            
            % the offsets of the 1d line connectivity array. Since all
            % lines only contain 2 points this is always 2,4,6, ...
            linesoff = (1:numpoints-1)*2;

            
            % start a new xml doc
            doc = VtkWriter.make_vtk_xml('PolyData');
            n_docNode = doc.getDocumentElement;
            n_polydata = doc.createElement('PolyData');
            n_docNode.appendChild(n_polydata);
            
            n_piece = doc.createElement('Piece');
            n_piece.setAttribute('NumberOfPoints', num2str(numpoints));
            n_piece.setAttribute('NumberOfVerts', '0');
            n_piece.setAttribute('NumberOfLines', num2str(numpoints-1));
            n_piece.setAttribute('NumberOfStrips', '0');
            n_piece.setAttribute('NumberOfPolys', '0');
            n_polydata.appendChild(n_piece);
            
            n_pointdata = doc.createElement('PointData');
            n_pointdataarray = VtkWriter.make_dataarray(doc, ...
                states_array(:,5), 'cost', 'Float32');
            n_pointdata.appendChild(n_pointdataarray);
            n_pointdataarray = VtkWriter.make_dataarray(doc, ...
                (0:numpoints-1)', 'index', 'Int32');
            n_pointdata.appendChild(n_pointdataarray);
            n_piece.appendChild(n_pointdata);
            
            n_points = doc.createElement('Points');
            n_pointsarray = VtkWriter.make_dataarray(doc, ...
                states_array(:,1:3), 'Points', 'Float32');
            n_points.appendChild(n_pointsarray);
            n_piece.appendChild(n_points);
            
            n_lines = doc.createElement('Lines');
            n_linesconnectivity = VtkWriter.make_dataarray(doc, ...
                linesconn, 'connectivity', 'Int32');
            n_linesoffsets = VtkWriter.make_dataarray(doc, ...
                linesoff', 'offsets', 'Int32');
            n_lines.appendChild(n_linesconnectivity);
            n_lines.appendChild(n_linesoffsets);
            n_piece.appendChild(n_lines);
            
            fname_out = VtkWriter.validate_filename(fname, '.vtp');
            xmlwrite(fname_out, doc);
            
        end
        
        function [ ] = write_map(map, map_zero, map_res, fname)
            % WRITE_MAP Function to write an occupancy grid to vtk.
            % Arguments:
            %   - map: The map as a N*M*P array.
            %   - map_zero: The zero point of the map (e.g. [0 0 0])
            %   - map_res: The size of a cell (e.g. [1 1 1])
            %   - fname: the filename. if the filename does not end in .vtr
            %           the ending is added automatically.
            
            % make sure that the resolution is valid
            if any(map_res <= 0)
                error('map_res must only contain positive values');
            end
            
            % compute the extent of the grid
            map_size = size(map) .* map_res;
            extent = zeros(1,6);
            extent(1:2:end) = map_zero;
            extent(2:2:end) = map_zero + map_size;
            
            
            % build up the extent string (basically contains the number of
            % cells)
            map_size = zeros(1,6);
            map_size(2:2:end) = size(map);
            map_extentstr = strtrim(sprintf(' %g', map_size));
            
            
            % start a new xml doc
            doc = VtkWriter.make_vtk_xml('RectilinearGrid');
            n_docNode = doc.getDocumentElement;
            n_grid = doc.createElement('RectilinearGrid');
            n_grid.setAttribute('WholeExtent', map_extentstr);
            n_docNode.appendChild(n_grid);
            
            n_piece = doc.createElement('Piece');
            n_piece.setAttribute('Extent', map_extentstr);
            n_grid.appendChild(n_piece);
            
            n_celldata = doc.createElement('CellData');
            n_celldata.setAttribute('Scalars', 'occupancy');
            n_celldataarray = VtkWriter.make_dataarray(doc, ...
                map(:), 'occupancy', 'Int8');
            n_celldata.appendChild(n_celldataarray);
            n_piece.appendChild(n_celldata);
            
            n_coordinates = doc.createElement('Coordinates');
            for i = 1:3
                values = linspace(extent(2*i-1), extent(2*i), ...
                    size(map, i) + 1)';
                n_coordarray = VtkWriter.make_dataarray(doc, values, ...
                    char(119+i), 'Float32');
                n_coordinates.appendChild(n_coordarray);
            end
            
            n_piece.appendChild(n_coordinates);
            
            % validate the filename and write the outputfile
            fname_out = VtkWriter.validate_filename(fname, '.vtr');
            xmlwrite(fname_out, doc);   
            
            
        end
    end
end