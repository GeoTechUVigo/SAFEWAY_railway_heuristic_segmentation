function [cabling] = create_cabling_seeds(cloud, rough_cabling, rails_pts, masts_centres)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    % Align cloud
    rails_pca   =   pca(rails_pts);
    cloud_loc   =   cloud.Location * rails_pca;
    % Define the location of the masts
    masts_loc   =   cat(1, masts_centres{:}) * rails_pca;
    % Select rough cabling points
    all_cable_pts  	=   cloud_loc(rough_cabling,:);
    
    
    CATINVER	=   cell(length(masts_centres),1);
    RETURN      =   CATINVER;
    CONTACT     =   CATINVER;
    CATENARY    =   CATINVER;
    TENSORS     =   CATINVER;
%     figure
    for i = 1:length(masts_centres)

        % Selecta a region perp to the rails around the mast
        mast_neigh_id   =   abs(all_cable_pts(:,1) - masts_loc(i,1)) < 2 &...
                            abs(all_cable_pts(:,2) - masts_loc(i,2)) < 6;
                        
        mast_neigh_id   =   rough_cabling(mast_neigh_id);
        mast_neigh_pts 	=   cloud_loc(mast_neigh_id,:);

        seeds       =   rangesearch(mast_neigh_pts(:,1:2), ...
                        masts_loc(i,1:2), 1);
        mast_clust  =   one_region_growing(mast_neigh_pts(:,1:2), 0.3, ...
                        mast_neigh_pts(seeds{1},1:2));
        
        mast_neigh_id   =   mast_neigh_id(mast_clust == 1);        
        mast_neigh_pts 	=   cloud_loc(mast_neigh_id,:);
        
        % Principal component analysis for each point
        id  =   rangesearch(mast_neigh_pts,mast_neigh_pts,0.2);
    
        ppal_dir    =   zeros(numel(id),3);
        normal      =   zeros(numel(id),3);
        
        a1d = zeros(numel(id),1);   a2d = a1d;  a3d = a1d;
        for j = 1:numel(id)
            points_j            =   mast_neigh_pts((id{j}),:);
            [autovec,~,lam]     =   pca(points_j,'Economy', false); 
            
            [a1d(j), a2d(j), a3d(j)]    =   dimensionality(lam); 
             
            ppal_dir(j,:)   =   autovec(:,1);
            normal(j,:)     =   autovec(:,3);
        end
        %% Identify whether there are tensors in the masat
        cond_A      =   normal(:,2) > 0.9;
        cond_B      =   (a3d > 2*a1d & a3d>2*a2d);
%         figure; pcshow(mast_neigh_pts(cond_B|cond_A,:), 'markersize', 20)
        if sum(cond_A|cond_B) > 100
            T_ids           =   find(cond_B);
            [T_ids,T_clust]	=   remove_noise(mast_neigh_pts,T_ids, .1, 30);
            num_clusts      =   unique(T_clust);
            if length(num_clusts) > 2
                [C,V]       =   groupcounts(T_clust);
                [~,keep_v]  =   maxk(C, 2);
                keep_v      =   V(keep_v);
                num_clusts  =   num_clusts(ismember(num_clusts,keep_v));
            end
            all_T   =   [];
            for c = 1:length(num_clusts)
                c_i         =   num_clusts(c);
                T_centre    =   mean(mast_neigh_pts(T_ids(T_clust==c_i),:));
%                 hold on; pcshow(tensors_centre, 'r', 'markersize', 400)
%                 hold on; pcshow(tensors_centre+[0.52,0,0], 'g', 'markersize', 400)
                T_c     =   pdist2(mast_neigh_pts, T_centre);
                T_p     =   point_to_plane_distance(mast_neigh_pts,...
                            [0 1 0], T_centre);
                T_c 	=   find(T_c < 0.6 & T_p < 0.2);
                all_T	=   [all_T; T_c];
            end
            if length(all_T) > 400            
                % Store tensor points
                TENSORS{i}	=   mast_neigh_id(all_T);
                % Remove tensor points for the following steps
                rem_T       	=   true(length(mast_neigh_pts),1);
                rem_T(all_T)    =   false; 
                mast_neigh_pts  =   mast_neigh_pts(rem_T,:);
                mast_neigh_id   =   mast_neigh_id(rem_T);
                ppal_dir        =   ppal_dir(rem_T,:);
            end
        end
        
        %% Separate mast cantilever from cabling defining a condition for 
        % perpendicularity
        cond_perp  =   find(ppal_dir(:,1) < 0.2);
        perp_pts   =   mast_neigh_pts(cond_perp,:);

        % 
        clust_ids   =   remove_noise(mast_neigh_pts,cond_perp, 0.3, 100);
        catinver    =   mast_neigh_id(clust_ids);
        
        % Store catinver points
        CATINVER{i}    =   catinver;
        %% Separate wires in the XY plane
        cables_loc  =    ~ismember(mast_neigh_id, catinver) & ...
                            ppal_dir(:,1)' > 0.8;
        cables_pts  =    mast_neigh_pts(cables_loc,:);
        cables_ids  =    mast_neigh_id(cables_loc);
        
        divide_return   =   kmeans(cables_pts(:,2), 2);
        
        % The return cable will be closer to the mast
        c1_2D  =   mean(cables_pts(divide_return == 1, 1:2));
        c2_2D  =   mean(cables_pts(divide_return == 2, 1:2));
        
        [~,r_id]    =   min(vecnorm([c1_2D;c2_2D] - masts_loc(i,1:2),2,2));
        return_wire =   cables_ids(divide_return == r_id);
        
        [return_wire, clusts]  =   remove_noise(cloud_loc, return_wire, 0.3, 15); 
        num_c       =   unique(clusts);
        keep_clust  =   false(length(num_c),1);
        for c = 1:length(num_c)
            clust_c     =   return_wire(clusts == num_c(c));
            [pca_return,~,lam]  =   pca(cloud_loc(clust_c,:));
            
            [a1d, a2d, a3d]    =   dimensionality(lam);
            
            if pca_return(1,1) > 0.9 && a1d > 2*a3d && a1d>2*a2d
                keep_clust(c) = true;
            end
        end
        keep_clust      =   num_c(keep_clust);
        return_wire     =   return_wire(ismember(clusts, keep_clust));
        
        % Store return wire points
        RETURN{i}      =   return_wire;
        %% Separate the rest of the cables into catenary (top) and contact 
        % (bottom) wires
        other_wires         =   cables_ids(divide_return ~= r_id);
        other_wires_pts     =   cloud_loc(other_wires,:);
        divide_catenaty     =   kmeans(other_wires_pts(:,3), 2);
        
        group_1         =   (divide_catenaty==1);
        h_1             =   other_wires_pts(find(group_1,1),3);
        h_2             =   other_wires_pts(find(~group_1,1),3);
        [~, cat_id]     =   max([h_1,h_2]);
        cat_wire        =   other_wires(divide_catenaty == cat_id);
        cont_wire       =   other_wires(divide_catenaty ~= cat_id);
        
        % Remove noise points
        cat_wire        =   remove_noise(cloud_loc, cat_wire, 0.2, 15);
        cont_wire       =   remove_noise(cloud_loc, cont_wire, 0.2, 15);
%         

        % Store catenary and contact wire points
        CONTACT{i}     =   cont_wire;
        CATENARY{i}    =   cat_wire;

        %% Visualize results
%         hold on; pcshow(cloud.Location(cat_wire,:), 'r', 'markersize', 50);
%         hold on; pcshow(cloud.Location(return_wire,:),'c','markersize',50);
%         hold on; pcshow(cloud.Location(cont_wire,:), 'g', 'markersize',50);
%         hold on; pcshow(cloud.Location(catinver,:), 'w', 'markersize',10);
%         hold on; pcshow(cloud.Location(TENSORS{i},:), 'w', 'markersize',10);
%         cloud_i = abs(cloud_loc(:,1) - masts_loc(i,1)) < 2 & ...
%                         abs(cloud_loc(:,2) - masts_loc(i,2)) < 6;
%         hold on; pcshow(cloud.Location(cloud_i,:), cloud.intensity(cloud_i), 'markersize', 1);
    end
   
    cabling.catinver        =   cat(2,  CATINVER{:});
    cabling.return_cable    =   cat(2,  RETURN{:});
    cabling.contact_wire    =   cat(2,  CONTACT{:});
    cabling.catenaty        =   cat(2,  CATENARY{:});
    cabling.tensors         =   cat(2,  TENSORS{:});
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