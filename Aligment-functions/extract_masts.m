function [full_mast_poles, traffic_lights] = extract_masts(notTrack, rails_pts)%[r_masts_ids, l_masts_ids] = extract_masts(notTrack, rails_pts,traj)
% Function that segments the mast poles points and traffic lights from the
% non-track points of a point cloud.
% Inputs:
% - notTrack: pointCloud_ object containing the non-track points.
% - rails: 3D array containing the coordinates of the rails.
% Outputs:
% - full_mast_poles: cell array containing the indices (referred to 
%                   notTrack cloud) of the mast poles points.
% - traffic_lights: cell array containing the indices (referred to 
%                   notTrack cloud) of the traffic lights points.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Align the cloud and the rails points to the principal components of  
    % the rails, and centre them to the midpoint of the rails section
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    notTrack_loc    =   notTrack.Location;
    rails_pca       =   pca(rails_pts, 'Economy', false);
    notTrack_loc    =   notTrack_loc * rails_pca;
    rails_pts       =   rails_pts * rails_pca;
%     traj_loc        =   traj.points * rails_pca;
    
    centre_pt       =   mean(rails_pts);
    notTrack_loc    =   notTrack_loc - centre_pt;
    rails_pts       =   rails_pts - centre_pt;
%     traj_loc        =   traj_loc - centre_pt;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Perform an initial detection of masts filtering by point height
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    heights     =   notTrack_loc(:,3);
    min_h       =   min(rails_pts(:,3))+0.5;
    max_h       =   min_h + 5;
    idx         =   find((heights > min_h) & (heights < max_h));

    initial_detect  =   select(notTrack, idx);
%     
%     figure
%     pcshow(notTrack_loc, notTrack.intensity)
%     hold on; pcshow(notTrack_loc(idx,:),'w')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Rasterise initial detected points and neglect pixels with few points
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    r   =   raster2DANM(initial_detect, 0.3);
    I   =   r.nPoints_image;
    I(I < 200) = 0;%Neglect small clusters

    cc  =   bwconncomp(I);

    cluster_idx = cell(numel(cc.PixelIdxList),1);
    for i = 1:numel(cc.PixelIdxList)
        aux_id      =   ismember(r.indices, cc.PixelIdxList{i});
        aux_id_2    =   cell2mat(r.parent_idx(aux_id)); %ids en cloud
         cluster_idx{i}     =   idx(aux_id_2);
    end

    % Delete empty clusters 
    cluster_idx     =   cluster_idx(~cellfun(@isempty, cluster_idx));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Identify pole-like objects. For each cluster find the Principal 
    % component coefficients and store those that satisfy (1) vertical 
    % component close to unity, (2) linear dimensionality analysis, (3) the
    % point cover all the range defined for the initial detection, and (4)
    % there are at least 900 points.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    poles_ids   =   false(length(cluster_idx),1);    
%     figure;
    for i = 1:numel(cluster_idx)
        pts     =   notTrack_loc(cluster_idx{i},:);
        [vecs,~,lam]    =   pca(pts, 'Economy', false); 
%       close
%       figure;   hold on; pcshow(pts,rand(1,3), 'markersize', 30)
        
        a1d  =   (sqrt(lam(1)) - sqrt(lam(2))) / sqrt(lam(1));
        a2d  =   (sqrt(lam(2)) - sqrt(lam(3))) / sqrt(lam(1));
        a3d  =   sqrt(lam(3)) / sqrt(lam(1));
             
%         aligned_range  =   range(pts*vecs);
        cond_1  =   vecs(3,1) > 0.85;
        cond_2  =   a1d > a2d*4 & a1d > a3d*4;
        cond_3  =   min(pts(:,3)) < min_h +.4 & max(pts(:,3)) > max_h -.3;
%         cond_4  =   aligned_range(2) < 0.6 & aligned_range(3) < 0.6;
        cond_4  =   length(pts) > 900;
        if cond_1 && cond_2 && cond_3 && cond_4
            poles_ids(i) = true;
        end
    end
%     close
    cluster_idx     =   cluster_idx(poles_ids);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Identify all the points of the poles by fitting a line to the initial 
    % detected clusters and filtering by point to line distance. Then 
    % neglect the poles which are non continuous or short
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    poles_centre    =   cellfun(@(x) mean(notTrack_loc(x,:)),...
                            cluster_idx, 'UniformOutput', false);
    % Relocate centre if found extra points (some masts have cylinders 
    % attached)
    range_mast  =   cellfun(@(x) range(notTrack_loc(x,1:2)) > 1,...
                            cluster_idx, 'UniformOutput', false); 
%     relocated   =   false(length(poles_centre),1);
    if any(cat(1, range_mast{:}), 'all')
        fat_masts   =   find(cat(1, range_mast{:}));
        num_masts   =   length(range_mast);
        for f = 1:length(fat_masts)
%             f_i     =   round(fat_masts(f)/2);
            f_i     =   fat_masts(f);
            if f_i > num_masts; f_i = f_i - num_masts; end
            mast_f  =    notTrack_loc(cluster_idx{f_i},:);
            [N, x_edges, ~]  =   histcounts2(mast_f(:,1), mast_f(:,2));
            [XX,~]     =   find(N == max(N,[],'all'));
            relocate_x  =   x_edges(XX +1);
            relocate_y  =   mean(mast_f(:,2));
            relocate_z  =   mean(mast_f(:,3));
            poles_centre{f_i} =   [relocate_x, relocate_y, relocate_z];
            
            % Delete some points
%             D = pdist2(mast_f(:,1:2), poles_centre{f_i}(:,1:2));
%             cluster_idx{f_i}    =   cluster_idx{f_i}(D < 1.2);
%             relocated(f_i)      =   true;
        end
    end
    
    
    % Actual pole points computed as point to line distance
    poles_pca    =   cellfun(@(x) pca(notTrack_loc(x,:)), cluster_idx, ...
                    'UniformOutput', false);
    poles_pca    =   cellfun(@(x) x(:,1)', poles_pca,...
                    'UniformOutput', false);
    poles_top    =   cellfun(@(x,y) x + y, poles_centre, poles_pca, ...
                    'UniformOutput', false);  
    full_poles	=	cellfun(@(x,y) find(point_to_line_distance(...
                        notTrack_loc,x,y)<0.45), poles_centre, ...
                        poles_top, 'UniformOutput', false);
                    
%         figure; hold on
    for i = 1:length(cluster_idx)
        pole_pts        =   notTrack_loc(cluster_idx{i},:);
        neighbour       =   notTrack_loc(full_poles{i},:);
        pole_clusts     =   one_region_growing(neighbour,0.3, pole_pts);
        full_poles{i}   =   cat(1,full_poles{i}(pole_clusts==1), ...
                            cluster_idx{i});
%         hold on; pcshow(notTrack_loc(full_poles{i},:), 'markersize', 50)
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Finally, select only the poles that are tall enough to be a mast
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    min_height_mast =   6.5;
    min_height_TL   =   4.5;
    
    full_poles_pts	=	cellfun(@(x) notTrack_loc(x,:), full_poles,...
                        'UniformOutput', false);
    poles_range     =	cellfun(@(x) range(x(:,3)), full_poles_pts);
    
    full_mast_poles	=   full_poles(poles_range >= min_height_mast);    
    cond_TL         =   poles_range < min_height_mast ...
                        & poles_range >= min_height_TL;
                    
    traffic_lights  =   full_poles(cond_TL);
    if sum(cond_TL) > 0         
        centre_TL   =   poles_centre(cond_TL);
        top_TL      =   poles_top(cond_TL);
        full_TL     =	cellfun(@(x,y) find(point_to_line_distance(...
                            notTrack_loc,x,y)<1.5), centre_TL, ...
                            top_TL, 'UniformOutput', false);
        for i = 1:length(traffic_lights)
            TL_pts      =   notTrack_loc(traffic_lights{i},:);
            TL_neigh    =   notTrack_loc(full_TL{i},:);
            extend_TL   =   one_region_growing(TL_neigh,...
                            0.15, TL_pts);
            traffic_lights{i}   =   full_TL{i}(extend_TL==1);
%             figure; pcshow(TL_neigh); hold on; pcshow(TL_neigh(extend_TL==1,:))
%             close
        end
    end
    
    
% a=0;

%     %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Differenciate left and right poles and remove masts that are too 
%     % close to each other as these could be bridge pillards, columns, etc.
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     traj_points      =   cellfun(@(x) (knnsearch(traj_loc, x)),  ...
%                             poles_centre,'UniformOutput', false);
%     % Separate left and right poles
%     left_poles_id = [];    right_poles_id = [];
%     for i = 1:length(poles_centre)
%         pt  =   poles_centre{i};
%         
%         try
%             v1  =   traj_loc(traj_points{i},:);
%             v2  =   traj_loc(traj_points{i}+1,:);
%             normal  =   cross((v2-v1), (pt-v1));
%         catch
%             v1  =   traj_loc(traj_points{i},:);
%             v2  =   traj_loc(traj_points{i}-1,:);
%             normal  =   cross((v1-v2), (pt-v1));
%         end            
% 
%         
%         if normal(3) > 0 % Left 
%             left_poles_id  =   [left_poles_id; i];
%             if length(left_poles_id)>1
%                 a=0;
%             end
%         else 
%             right_poles_id =   [right_poles_id; i];
%             if length(right_poles_id)>1
%                 a=0;
%             end
%         end
%     end
%     l_masts_ids     =   full_poles(left_poles_id);
%     r_masts_ids     =   full_poles(right_poles_id);

%     i = length(poles_centre);
%     pt  =   poles_centre{i};
%     v1  =   traj_loc(traj_points{i}(1),:);
%     v2  =   traj_loc(traj_points{i}(2),:);
%     [~,s]        =   point_to_line_distance(pt,v2,v1);
%     if s > 0 % Right 
%         left_poles_id  =   [left_poles_id; i];
%     else 
%         right_poles_id =   [right_poles_id; i];
%     end
    
    
        
    % Determine which of the right poles are masts
%     poles_dist  =   0;
%     right_masts       =   false(1, numel(right_poles));
%     for i = 1:numel(right_poles)
%         j = right_poles_id(i);
%         prev_dist   =   poles_dist;
%         if i < numel(right_poles)
%             pole        =   poles_centre{j};
%             next_pole   =   poles_centre{right_poles_id(i+1)};
%             dif         =   next_pole - pole;
%             poles_dist  =   norm(dif);    
%             if or(poles_dist > 5, prev_dist > 5)
%                 right_masts(i)     =   true;
%             end
%         else
%             if prev_dist > 10 
%                 right_masts(i)     =   true;
%             end
%             break
%         end
%     end

%     if numel(right_masts) ~= 0
%         right_masts_ids   =   full_poles(right_poles_id(right_masts));
%         right_masts_ids   =   cellfun(@(x) notTrack_ids(x),right_masts_ids,...
%                             'UniformOutput',false);
%     else
%         right_masts_ids = {};
%     end
%     % Determine which of the left poles are masts
%     poles_dist  =   0;
%     left_masts       =   false(1, numel(left_poles));
%     for i = 1:numel(left_poles)
%         j = left_poles_id(i);
%         prev_dist   =   poles_dist;
%         if i < numel(left_poles)
%             pole        =   poles_centre{j};
%             next_pole   =   poles_centre{left_poles_id(i+1)};
%             dif         =   next_pole - pole;
%             poles_dist  =   norm(dif);    
%             if or(poles_dist > 5, prev_dist > 5)
%                 left_masts(i)     =   true;
%             end
%         else
%             if prev_dist > 10  
%                 left_masts(i)     =   true;
%             end
%             break
%         end
%     end

%     if numel(left_masts) ~= 0
%         left_masts_ids   =   full_poles(left_poles_id(left_masts));
%         left_masts_ids   =   cellfun(@(x) notTrack_ids(x),left_masts_ids,...
%                             'UniformOutput',false);
%     else
%         left_masts_ids = {};
%     end

end

