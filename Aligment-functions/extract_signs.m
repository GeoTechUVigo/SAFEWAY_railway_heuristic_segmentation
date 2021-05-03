function [sign_i, sign_type] = extract_signs(cloud_loc, seeds_cell, rails)
% Function that retrieves the whole sign objects extended from am initial
% detection.
% Inputs:
%  - cloud_loc: pointCloud_ object or 3D coordinates of a point cloud.
%  - signs_cell: cell array of the indices of the signs seed points.
%  - rails: array containing the 3D coordinates of the cloud rails.
% Outputs:
%  - sign _i: cell array containing the indices of the cloud corresponding 
%             to the whole traffic sign.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Check if the input cloud is a pointCloud or just coordinates, force
    % seeds_cell to be a cell and initialise outputs    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~isa(cloud_loc, 'double')
        cloud_loc   =   cloud_loc.Location;
    end
    if ~isa(seeds_cell, 'cell')
        seeds_cell  =   mat2cell(seeds_cell, 1);
    end
    sign_i      =   cell(1, length(seeds_cell));
    sign_type   =   zeros(1, length(seeds_cell));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Align cloud to rails and compute the principal components
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    cloud_loc   =   cloud_loc * pca(rails);
    rails_loc   =   rails * pca(rails);
    X_c         =	cloud_loc(:,1);
    Y_c         =	cloud_loc(:,2);
   
    pcas        =   cellfun(@(x) pca(cloud_loc(x,:), 'Economy', false),...
                            seeds_cell, 'UniformOutput', false);
    normals     =   cellfun(@(x) x(:,3), pcas, 'UniformOutput', false);
    lines       =   cellfun(@(x) x(:,1), pcas, 'UniformOutput', false);

    % Loop for each element in seeds_cell
    for i = 1:length(seeds_cell)
        seeds           =   seeds_cell{i};
        seeds_xyz       =   cloud_loc(seeds,:);  
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Eliminate extra points, noise. The knnsearch is needed because
        % sometimes a far away point moves the mean too far from the
        % objective cluster. Apply then a region growing clustering to 
        % separate each panel in the sign.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        mean_p      =   knnsearch(seeds_xyz(:,2:3), ...
                        mean(seeds_xyz(:,2:3)), 'K', 1);
        noise       =   one_region_growing(seeds_xyz(:,2:3), 0.7, ...
                        (seeds_xyz(mean_p,2:3)));         
        seeds       =   seeds(noise == 1);
        seeds_xyz   =   seeds_xyz(noise == 1,:);

        seeds_clust   =   region_growing_cluster(seeds_xyz(:,1:2), 0.05);
        [seeds_c, seeds_v]  =   groupcounts(seeds_clust);
        
        keep_v      =   seeds_v(seeds_c > max(seeds_c) /3 );        
        keep_h      =   true(1, length(keep_v));    
        
        mean_h          =   zeros(1, length(keep_v));       
        xy_range_clust  =   zeros(length(keep_v),2);    
        for c = 1:length(keep_v) % This eliminates cabling and other noise
            coord = seeds_xyz(seeds_clust == keep_v(c),:);
            mean_h(c)       =   mean(coord(:,3)) - mean(rails_loc(:,3));
            xy_range_clust(c,:)     =   range(coord(:,1:2), 1);
            if mean_h(c) > 5 || sum(xy_range_clust(c,:) < 0.07) == 2
                keep_h(c) = false;
            end
        end
        
        keep_v      =   keep_v(keep_h);
        keep_seeds  =   ismember(seeds_clust, keep_v); 
        seeds       =   seeds(keep_seeds);
        
        if isempty(seeds);     continue;         end
        
        seeds_xyz   =   seeds_xyz(keep_seeds,:);
        seeds_clust =   seeds_clust(keep_seeds);
        seeds_range =   range(seeds_xyz,1);
        
        % Check if the seed points are a sign by determining if their
        % normal is a vector pointing towards the y-direction
        condition_1     =   abs(normals{i}(1)) > 0.85;
        condition_2     =   lines{i}(3) > 0.85;
        condition_3     =   sum(seeds_range(:,1:2) < 0.1) < 2;
        if sum([condition_1, condition_2, condition_3]) < 2; continue; end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        % Identify the surrounding points of the sign and define the top
        % points. Perform a one region grouwing to neglect isolated points.        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        X           =   mean(seeds_xyz(:,1));
        Y           =   mean(seeds_xyz(:,2));
        radius      =   (max(range(seeds_xyz(:,1:2))) * 1.05)/ 2;
        radius      =   max([radius, 0.15]);
        object      =   find((X_c - X).^2 + (Y_c - Y).^2 <= radius); 
        object_xyz  =   cloud_loc(object,:);
        
        
        surroundings        =   object(~ismember(object, seeds));
        surroundings_xyz    =   cloud_loc(surroundings,:);

% FIGURE% %
%             figure; 
%%%             hold on;pcshow(cloud_loc); 
%             hold on; pcshow(cloud_loc(surroundings, :),'w')
%             hold on; pcshow(seeds_xyz,seeds_clust,'markersize', 50)

        % Identify the type of sign (pole sign or sign attached to a mast)

        % Perform one region growing to check if the detected sign is a
        % mast
        sign_idx    =   one_region_growing(object_xyz, 0.15, seeds_xyz);
        sign_xyz    =   object_xyz(sign_idx==1,:);
        sign_idx    =   object(sign_idx==1);
        
        % Define the top points
        top_points	=   sign_xyz(sign_xyz(:,3) > max(seeds_xyz(:,3)) + ...
                        0.5 & sign_xyz(:,3) < max(seeds_xyz(:,3)) + 2.5,:);


        top_H_diff      =   max(seeds_xyz(:,3)) - max(sign_xyz(:,3));
        range_xyz       =   range(sign_xyz);
            
        if  length(top_points) < 150 ||  abs(top_H_diff) < 1 || range_xyz(:,3) < 6 %top_pca(3,1) < 0.85 ||

            % In this case the signal is a post sign because all the points
            % avobe the sign are cables. Thus, all is considered a sign
            pca_pole    =   pca(cloud_loc(sign_idx,:),'Economy', false);
            normal_pole =   pca_pole(:,3);
            min_pole_pt     =   sign_xyz(sign_xyz(:,3)== min(sign_xyz(:,3)),:);
            close_rail_pt   =   rails_loc(knnsearch(rails_loc, min_pole_pt),:);
            min_H_pole      =   min_pole_pt(3) - close_rail_pt(3);
            if min_H_pole < 0 && min_H_pole > -0.5
                range_H_pole    =   max(sign_xyz(:,3)) - close_rail_pt(3);
            else
                range_H_pole    =   range_xyz(3);
            end
            
            if (range_xyz(2) < 1.5 && min_H_pole < 0.5) 
                if (range_H_pole > 2 && range_xyz(2) > 0.7 && normal_pole(1) > 0.85) % Tall pole sign
                    sign_i{i}       =   sign_idx;
                    sign_type(i)    =   1;
                elseif (range_H_pole > 5 && sum(pca_pole > 0.85,'all')...
                        == 3) && sum(range_xyz(1:2) < 1.2) == 2% Traffic light
                    
                    % some traffic light seeds are only in the surface so
                    % the radius is small.
                    X   =   mean(sign_xyz(:,1));
                    Y   =   mean(sign_xyz(:,2));
                    large_surr_ids  =   find((X_c - X).^2 + (Y_c - Y).^2 <= 4^2); 
                    large_surr_xyz  =   cloud_loc(large_surr_ids,:);   
                    traffic_light   =   one_region_growing(large_surr_xyz, 0.08, sign_xyz);
                    pole_sign       =   large_surr_ids(traffic_light==1);
                    if sum(range(cloud_loc(pole_sign,1:2)) > 3) > 0
                        continue
                    end
                    sign_i{i}       =   pole_sign;
                    sign_type(i)    =   2;
                elseif(range_H_pole < 2.2 && sum(range_xyz(1:2)<.9)== 2 ...
                       && sum(pca_pole > 0.85,'all') == 3) % Small sign
                    sign_i{i}       =   sign_idx;
                    sign_type(i)    =   3;
                end
%                 hold on; pcshow(cloud_loc(sign_i{i},:), 'r', 'markersize',50)
            end
        else
            % Here the signs are attached to a mast and they need to be
            % separated from the mast points.    
            num_signs           =   unique(seeds_clust);
            clust_normals       =   zeros(length(num_signs),3); 
            mean_pt             =   zeros(length(num_signs),3); 
            sign_j              =   cell(1, length(num_signs));
            for j = 1:length(num_signs)
                jj = num_signs(j);
                seeds_clust_xyz     =   seeds_xyz(seeds_clust == jj,:);
                if numel(seeds_clust_xyz) < 9
                    continue
                end
                clust_pca           =   pca(seeds_clust_xyz,'Economy', false);
                clust_normals(j,:)  =   clust_pca(:,3);
                mean_pt(j,:)        =   mean(seeds_clust_xyz);
                % Represent this plane in a figure
                plane = createPlane(mean(seeds_clust_xyz), clust_normals(j,:));
%                 figure; pcshow(seeds_xyz,'r','markersize',50); hold on; pcshow(surroundings_xyz,'w'); hold on
%                 drawPlane3d(plane,rand(1,3));
                condition_3     =   abs(plane(6)) > 0.8 && abs(plane(8)) > 0.8;
                condition_4     =   abs(plane(5)) > 0.8 && abs(plane(9)) > 0.8;
                if ~condition_3 && ~condition_4 && abs(clust_normals(j,1)) < 0.8
                    continue
                end
                rep_normals     =   repmat(clust_normals(j,:),length(surroundings),1);
                pt_to_plane_d   =   abs(dot(surroundings_xyz - mean_pt(j,:), rep_normals,2));


                sign_j_id       =   surroundings(pt_to_plane_d < 0.07);
                sign_j_xyz      =   surroundings_xyz(pt_to_plane_d < 0.07,:);    

                H_LIM =  sign_j_xyz(:,3) > min(seeds_xyz(:,3)) - 0.1 & ...
                         sign_j_xyz(:,3) < max(seeds_xyz(:,3)) + 0.1  ;   
                sign_j{j}       =   sign_j_id(H_LIM);   
            end
            sign_j = cat(1, sign_j{:});
            if isempty(sign_j); continue; end
            sign_i_xyz  =   cloud_loc(cat(1, sign_j, seeds),:);
            sign_i_x    =   sign_i_xyz(:,2);
            min_H       =   min(sign_i_xyz(:,3)) - max(rails_loc(:,3));
            range_x     =   range(sign_i_x);
            
            b1      =   min(sign_i_x);   b2  =   b1 + range_x / 3;             
            b4      =   max(sign_i_x);   b3  =   b4 - range_x / 3;
            bin_c   =   histcounts(sign_i_x,'BinEdges',[b1 b2 b3 b4]);
            hollow_cond    =   bin_c(2) > mean([bin_c(1),bin_c(3)]) * 0.5;
            % Las señales miden 1m de ancho normalmente y no están "huecas"
            if min_H < 4 && range_x > 0.40 && hollow_cond
                sign_i{i}       =   cat(1, sign_j, seeds);
                sign_i{i}       =   unique(sign_i{i});
                sign_type(i)    =   4;
%                 figure;
%                 hold on; pcshow(cloud_loc(sign_i{i},:), 'r', 'markersize',50)
%                 oo =0;
            end
        end
        
    end

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    