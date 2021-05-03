 classdef raster2D
    
    
    properties
        parent_cloud;
        parent_idx;
        indices;
        XLimit;
        YLimit;
        intensity_image;
        angle_image;
        nPoints_image;
        height_image; %Ranges
        height2_image; %Averages
        height3_image %max
        %color_image;
    end
    
    methods
        function obj = raster2D(cloud, grid) %Constructor de la clase.
            
            obj.parent_cloud = cloud.Location;
            min_x = min(cloud.Location(:,1)); max_x = max(cloud.Location(:,1));
            min_y = min(cloud.Location(:,2)); max_y = max(cloud.Location(:,2));

 

            n_largo = ceil((max_x - min_x) / grid);
            n_ancho = ceil((max_y - min_y) / grid);

 
            % Original
%             paso_x = (max_x - min_x) / n_largo;
%             paso_y = (max_y - min_y) / n_ancho;
% 
%             dim_x = round((cloud.Location(:,1)-min_x) / paso_x);
%             dim_y = round((cloud.Location(:,2)-min_y) / paso_y);

            % Modificación
            paso_x = grid;
            paso_y = grid;

            dim_x = ceil((cloud.Location(:,1)-min_x) / paso_x);
            dim_y = ceil((cloud.Location(:,2)-min_y) / paso_y);

 

            %Move points in voxel 0 to voxel 1 in each dimension
            dim_x(dim_x==0) = 1;
            dim_y(dim_y==0) = 1;

 

            idx = dim_y + (dim_x - 1) * n_ancho;
            parent_id = (1:size(idx,1))';
            pixel = [obj.parent_cloud, double(cloud.intensity), double(cloud.angle) parent_id];
            dim = [dim_y, dim_x]; 
            [idx, shift_index] = sortrows(idx);
            dim = dim(shift_index,:); 
            pixel_points = pixel(shift_index,:);
            [populated_pixels, first_point] = unique(idx);
            dim = dim(first_point,:); 
            %Get number of points of each voxel. This information will be useful for
            %speed up the feature calculation.
            populated_pixels(1:end-1, 2) = first_point(2:end)-first_point(1:end-1);
            populated_pixels(end,2) = (size(pixel,1)+1) - first_point(end);

 

            nPixels = size(populated_pixels,1);
            lastPoint = 0;
            acc_int = zeros(n_ancho , n_largo);
            angle = zeros(n_ancho, n_largo);
            numberPoints = zeros(n_ancho, n_largo);
            height = zeros(n_ancho, n_largo);
            height2 = zeros(n_ancho, n_largo);
            height3 = zeros(n_ancho, n_largo);
            parent_idx = cell(nPixels,1);
            
            for i = 1:nPixels
                nPoints = populated_pixels(i,2);
                if lastPoint == 0
                    pixel_data = pixel_points(1:nPoints,:);
                else
                    pixel_data = pixel_points(lastPoint+1:lastPoint+nPoints,:);
                end
                    lastPoint = lastPoint + nPoints;
                    acc_int(dim(i,1), dim(i,2)) = sum(pixel_data(:,4) / nPoints); 
                    numberPoints(dim(i,1), dim(i,2)) = nPoints;
                    angle(dim(i,1), dim(i,2)) = mean(pixel_data(:,5));
                    height(dim(i,1), dim(i,2)) = range(pixel_data(:,3));
                    height2(dim(i,1), dim(i,2)) = mean(pixel_data(:,3));
                    height3(dim(i,1), dim(i,2)) = max(pixel_data(:,3));
                    parent_idx{i} = pixel_data(:,6); 
                    
            end
            
            %obj.color_image = color_image;
            obj.parent_idx = parent_idx;
            obj.indices = populated_pixels(:,1); 
            obj.XLimit = n_largo;
            obj.YLimit = n_ancho;
            
            %Descomentar las siguientes líneas si hay saturación de
            %intensidad en algún punto. 
            obj.intensity_image = mat2gray(acc_int);
            
            obj.angle_image = angle;
            obj.nPoints_image = numberPoints;
            obj.height_image = height;
            obj.height2_image = height2;
            obj.height3_image = height3;
            
        end
        function pt_idx = get_pt_indices(this)
            pt_idx = zeros(length(this.parent_cloud),1);
            for i = 1:numel(this.parent_idx)
                pt_idx(this.parent_idx{i}) = this.indices(i);
            end
        end
        function color_image = get_color(this, cloud)
            nIndices = numel(this.indices);
            red = zeros(this.YLimit, this.XLimit);
            green = zeros(this.YLimit, this.XLimit);
            blue = zeros(this.YLimit, this.XLimit);
            color_image = zeros(this.YLimit, this.XLimit, 3);
            for i = 1:nIndices
                parent_id = this.parent_idx{i};
                colors = cloud.Color(parent_id,:);
                red(this.indices(i)) = mean(colors(:,1)) / 255;
                green(this.indices(i)) = mean(colors(:,2)) / 255;
                blue(this.indices(i)) = mean(colors(:,3)) / 255;
            end
            color_image(:,:,1) = red;
            color_image(:,:,2) = green;
            color_image(:,:,3) = blue;
        end
    end
end
 

