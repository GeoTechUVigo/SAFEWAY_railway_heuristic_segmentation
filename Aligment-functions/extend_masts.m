function [masts_ext] = extend_masts(cloud, rails_pts, masts_cell)
% Function that extracts the catinvers and cabling tensors from the masts 
% poles.
% Inputs:
% - cloud: whole pointCloud_.
% - rails_pts: array containing the coordinates of the rails for a section.
% - mast_poles: cell array output from extract_masts.
% Outputs:
% - masts_ext: struct containing
%       - catinver
%       - tensors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Align the cloud to the rails  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rails_pca   =   pca(rails_pts);
    cloud_loc   =   cloud.Location * rails_pca;
    
    % Define different masts parameters
    masts_pts 	=	cellfun(@(x) cloud_loc(x,:), masts_cell,...
                   	'UniformOutput', false);
    masts_top	=	cellfun(@(x) x(x(:,3) == max(x(:,3)),:), masts_pts,...
                    'UniformOutput', false);
    % Define the location of the masts
    masts_loc   =   cat(1, masts_top{:});
    mean_tops   =   mean(masts_loc(:,3));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Select top cloud points (i.e. cabling) and remove the mast poles from
    % them
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    high_ids  	=   find(cloud_loc(:,3) > mean_tops - 3 & ...
                    cloud_loc(:,3) < mean_tops + 0.5) ;
    high_pts    =   cloud_loc(high_ids,:);    
    % Remove mast points from selection
    all_masts_pts   =   cat(1, masts_pts{:});
    are_masts       =   ismember(high_pts, all_masts_pts, 'rows');
    high_pts        =   high_pts(~are_masts,:);
    high_ids        =   high_ids(~are_masts);   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % For each of the masts in the section, find the catinver and tensor
    % points. For this, cut a section of the top points and perform a
    % principal component analysis.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    CATINVER	=   cell(length(mean_tops),1);
    TENSORS     =   CATINVER; 
    for i = 1:length(masts_top)
        % Select a region perpendicular to the rails around the mast
        mast_neigh_id   =   abs(high_pts(:,1) - masts_loc(i,1)) < 2 &...
                            abs(high_pts(:,2) - masts_loc(i,2)) < 6;                        
        mast_neigh_id   =   high_ids(mast_neigh_id);
        mast_neigh_pts 	=   cloud_loc(mast_neigh_id,:);
        
        seeds       =   rangesearch(mast_neigh_pts(:,1:2), ...
                        masts_loc(i,1:2), 1);                    
        mast_clust  =   one_region_growing(mast_neigh_pts(:,1:2), 0.3, ...
                        mast_neigh_pts(seeds{1},1:2));
        
        mast_neigh_id   =   mast_neigh_id(mast_clust == 1);        
        mast_neigh_pts 	=   cloud_loc(mast_neigh_id,:);
        
        % Principal component analysis for each point
        neigh_cloud     =   pointCloud_(mast_neigh_pts);
        vx_neigh        =   Voxels(neigh_cloud, 0.02);
        vx_neigh_loc    =   vx_neigh.Location;
        id              =   rangesearch(vx_neigh_loc, vx_neigh_loc, 0.2);  
        
        ppal_dir	=   zeros(numel(mast_neigh_id),3);
        a1d         =   zeros(numel(mast_neigh_id),1);        
        normal      =   ppal_dir;   a2d	=	a1d;    a3d	=   a1d;
        for j = 1:numel(id)            
            neig_j	=   id{j};
            p_j   	=   vx_neigh.parent_pt_idx(neig_j);
            p_j    	=   cat(1,p_j{:});
            
            all_vx_pts      =   mast_neigh_pts(p_j,:);
            [autovec,~,lam]	=   pca(all_vx_pts,'Economy', false);
            [a1d(p_j), a2d(p_j), a3d(p_j)]    =   dimensionality(lam); 
             
            ppal_dir(p_j,:)   =   repmat(autovec(:,1)', [length(p_j), 1]);
            normal(p_j,:)     =   repmat(autovec(:,3)', [length(p_j), 1]);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Determine whether there are tensors in the mast
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        TENSORS{i,1}	=   [];
        cond_A      =   normal(:,2) > 0.9;
        cond_B      =   (a3d > 2*a1d & a3d>2*a2d);
        if sum(cond_A|cond_B) > 100
            T_ids           =   find(cond_B);
            [T_ids,T_clust]	=   remove_noise(mast_neigh_pts,T_ids, .2, 20);
            num_clusts      =   unique(T_clust);
            
            all_T           =   [];     all_pts         =   [];            
            for c = 1:length(num_clusts)
                c_i     =   num_clusts(c);
                T_cent	=   mean(mast_neigh_pts(T_ids(T_clust==c_i),:));                
                T_c_2D  =   pdist2(mast_neigh_pts(:,1:2), T_cent(1:2));
                T_c     =   pdist2(mast_neigh_pts, T_cent);
                T_p     =   point_to_plane_distance(mast_neigh_pts,...
                            [0 1 0], T_cent);
                T_c 	=   find(T_c_2D < 0.6 & T_c < 0.8 & T_p < 0.2);                
                pts     =   mast_neigh_pts(T_c,:);
                vec     =   pca(pts, 'Economy', false);
                if  vec(2,3) > 0.9 && length(pts) > 300
                    all_T	=   [all_T; T_c];
                    all_pts =   [all_pts; pts];
                end                
            end
            % Must be near the mast
            if ~isempty(all_T)
                top_pt 	=   masts_top{i};
                [~,D]	=   knnsearch(all_pts(:,1:2), top_pt(1:2));
                if  D < 0.8          
                    % Store tensor points
                    TENSORS{i,1}	=   mast_neigh_id(all_T)';
                    % Remove tensor points for the following steps
                    rem_T       	=   true(length(mast_neigh_pts),1);
                    rem_T(all_T)    =   false; 
                    mast_neigh_pts  =   mast_neigh_pts(rem_T,:);
                    mast_neigh_id   =   mast_neigh_id(rem_T);
                    ppal_dir        =   ppal_dir(rem_T,:);                
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Separate mast cantilever from cabling defining a condition for 
        % perpendicularity
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        cond_perp	=   find(ppal_dir(:,1) < 0.2);
        clust_ids   =   remove_noise(mast_neigh_pts,cond_perp, 0.3, 100);
        catinver    =   mast_neigh_id(clust_ids);
        
        % Store catinver points
        CATINVER{i,1}    =   catinver';

%         %% Visualize results
%         figure
%         hold on; pcshow(cloud.Location(mast_neigh_id,:))
%         hold on; pcshow(cloud.Location(CATINVER{i},:), 'w', 'markersize',10);
%         hold on; pcshow(cloud.Location(TENSORS{i},:), 'c', 'markersize',10);
%         hold on; pcshow(cloud.Location(masts_cell{i},:), cloud.intensity(masts_cell{i},:))
    end
   
    masts_ext.catinver        =   CATINVER;
    masts_ext.tensors         =   TENSORS;
end

%% AUXILIAR FUNCTIONS
function [ids, ids_clust] = remove_noise(cloud_loc, ids, r, c_limit)
    ids_clust   =   region_growing_cluster(cloud_loc(ids,:), r);
    [C,V]       =   groupcounts(ids_clust); 
    keep_v      =   V(C > c_limit); 
    keep_ids    =   ismember(ids_clust,keep_v);
    ids         =   ids(keep_ids);
    ids_clust   =   ids_clust(keep_ids);
end


function [a1d, a2d, a3d]    =   dimensionality(lamda)
    a1d	=   (sqrt(lamda(1)) - sqrt(lamda(2))) / sqrt(lamda(1));
    a2d =   (sqrt(lamda(2)) - sqrt(lamda(3))) / sqrt(lamda(1));
    a3d =   sqrt(lamda(3)) / sqrt(lamda(1));
end