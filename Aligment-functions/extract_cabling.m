function [cabling] = extract_cabling(cloud, all_ret_ids, all_cat_ids, rails)
%UNTITLED Summary of this function goes here
% Inputs:
% -	cloud: pointCloud_ object containing the non-track points. 
% -	ret_ids: indices of the points corresponding to feed and return wires.
% -	cat_ids: indices of the points corresponding to catenary, contact and dropper cables.
% -	rails: 3D array containing the coordinates of the section rails.
% Outputs:
% -	cabling:  struct containing the indices of the different wires in absolute indices:
%       -	return_wire
%       -	feed_wire
%       -	catenary
%       -	contact_wire
%       -	droppers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Align cloud to the rails
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rails_pca       =   pca(rails);
    cloud_loc       =   cloud.Location * rails_pca;
    all_ret_pts 	=   cloud_loc(all_ret_ids,:);
    all_cat_pts     =   cloud_loc(all_cat_ids,:);
    all_ids         =   [all_ret_ids, all_cat_ids];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % RETURN AND FEED WIRES. 
    % First voxelise return points cloud to speed up computations. 
    % Determine if there is more than one set of cables. For each set of 
    % cables, select the horizontal points only and separate the top (feed) 
    % and bottom (return) wires.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ret_cloud   =   pointCloud_(all_ret_pts);
    ret_vx      =   Voxels(ret_cloud, 0.1);

    if range(ret_vx.Location(:,2)) > 5
        num_cables  =   2;
        divide_y_ids  =   kmeans(ret_vx.Location(:,2),num_cables); 
    else
        num_cables  =   1;
        divide_y_ids  =   ones(length(ret_vx.Location),1); 
    end

    RETURN_ID  =   cell(num_cables,1);
    FEED_ID    =   cell(num_cables,1);
    for i = 1:2
        ret_ids     =   find(divide_y_ids == i);
        ret_pts     =   ret_vx.Location(ret_ids,:);
        r           =   rangesearch(ret_pts, ret_pts, 0.3); 
        ppal_dir    =   zeros(length(r),3);
        for j = 1:numel(r)
            neigh_j         =   ret_pts(r{j},:); 
            autovec         =   pca(neigh_j,'Economy', false); 
            ppal_dir(j,:)   =   autovec(:,1)';        
        end
        select_ids	=   abs(ppal_dir(:,1))> 0.9;
        ret_ids     =   ret_ids(select_ids);
        ret_pts     =   ret_pts(select_ids,:);

        if range(ret_pts(:,3)) > 1
            % In this case, there are two cables, the feed wire (top) and  
            % return wire (bottom)
            div_z_i	=   kmeans(ret_pts(:,3), 2);
            h1    	=   min(ret_pts(div_z_i == 1,3));
            h2     	=   min(ret_pts(div_z_i == 2,3));
            [~,r_i]	=   min([h1,h2]);
            
            return_ids  =   ret_ids(div_z_i == r_i);    
            feed_ids    =   ret_ids(div_z_i ~= r_i);    
            % Check if this division is correct.
            cond_1       =   range(cloud_loc(feed_ids,3)) > 1;
            cond_2       =   range(cloud_loc(return_ids,3)) > 1;
            if cond_1 || cond_2
                break_ids   =   kmeans(ret_pts(:,1), 2);
                return_ids  = [];	feed_ids    = [];
                for b_i = 1:2
                    sec_ids     =   find(break_ids == b_i); 
                    sec_pts     =   ret_pts(sec_ids,:);
                    if range(sec_pts(:,3)) > 1
                        sec_pts     =   sec_pts * pca(sec_pts); 
                        % Align the cables (se quedan tumbados)
                        div_z_i	=   kmeans(sec_pts(:,2), 2);
                        h1    	=   min(ret_pts(sec_ids(div_z_i == 1),3));
                        h2    	=   min(ret_pts(sec_ids(div_z_i == 2),3));                        
                        [~,r_i]	=   min([h1,h2]);
                        
                        return_bi   =   sec_ids(div_z_i == r_i);    
                        feed_bi     =   sec_ids(div_z_i ~= r_i);
                        return_ids  =   [return_ids; ret_ids(return_bi)];
                        feed_ids    =   [feed_ids; ret_ids(feed_bi)];
                    else
                        return_ids  =   [return_ids; ret_ids];
                    end
                end
            end

        else
            return_ids  =   ret_ids; 
            feed_ids    =   [];
        end
        RETURN_ID{i} 	=    return_ids;
        FEED_ID{i}      =    feed_ids;    
    end
    RETURN_ID   =   cat(1, RETURN_ID{:});
    FEED_ID     =   cat(1, FEED_ID{:});

    % Recover parent points
    RETURN_ID 	=   ret_vx.parent_pt_idx(RETURN_ID);
    FEED_ID 	=   ret_vx.parent_pt_idx(FEED_ID);
    RETURN_ID	=   all_ret_ids(cat(1, RETURN_ID{:}));
    FEED_ID 	=   all_ret_ids(cat(1, FEED_ID{:}));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Catenary, contact wire and droppers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Divide points if there ir more than one set of cables
    cat_cloud   =   pointCloud_(all_cat_pts);
    cat_vx      =   Voxels(cat_cloud, 0.1);

    if range(cat_vx.Location(:,2)) > 5
        num_cables  =   2;
        divide_y_ids  =   kmeans(cat_vx.Location(:,2),num_cables); 
    else
        num_cables  =   1;
        divide_y_ids  =   ones(length(cat_vx.Location),1); 
    end


    CATENARY_ID     =   cell(num_cables,1);
    CONTACT_ID      =   cell(num_cables,1);
    DROPPERS_ID  	=   cell(num_cables,1);
    for i = 1:2
        cat_ids     =   find(divide_y_ids == i);
        cat_pts     =   cat_vx.Location(cat_ids,:);
        r           =   rangesearch(cat_pts, cat_pts, 0.2); 
        ppal_dir    =   zeros(length(r),3);
        for j = 1:numel(r)
            neigh_j         =   cat_pts(r{j},:);    
            autovec         =   pca(neigh_j,'Economy', false); 
            ppal_dir(j,:)   =   autovec(:,1)';
        end
        
        % DROPPERS
        % Select droppers as those points with a vertical ppal direction
        droppers_ids    =   cat_ids(abs(ppal_dir(:,3))>0.5);
        droppers_pts   	=  	cat_vx.Location(droppers_ids,:);
        DROPPERS_ID{i}	=   droppers_ids;

        % Region growing cluster 
        drop_clusts     =   region_growing_cluster(droppers_pts, 2);
        drop_centr      =   zeros(max(drop_clusts), 3);
        for dc = 1:max(drop_clusts)   
            drop_pts            =    droppers_pts(drop_clusts==dc,:);
            drop_centr(dc,:)	=    mean(drop_pts,1);
        end
        % Select catenary and contact wires as the horizontal lines
        horz_id     =   abs(ppal_dir(:,1))>=0.95 & abs(ppal_dir(:,3))<0.3;
        horz_pts    =   cat_pts(horz_id,:);
        horz_id     =   cat_ids(horz_id);  

        % Find the nearest drop centre to each horizontal cable and 
        % determine whether the distance is positive or negative
        near_D_pt       =   knnsearch(drop_centr,horz_pts);
        vec_sign        =   sign(drop_centr(near_D_pt,3) - horz_pts(:,3));
        catenary_ids    =   horz_id(vec_sign > 0);
        contact_ids     =   horz_id(vec_sign < 0);

        % CATENARY WIRE
        % Delete far clusts
        catenary_pts    =    cat_vx.Location(catenary_ids,:);    
        CC      =   region_growing_cluster(catenary_pts, 0.2);    
        [C,V]   =   groupcounts(CC);
        big_c   =   ismember(CC, V(C > max(C)*0.8));
        big_pts =   catenary_pts(big_c , :);
        big_dir =   pca(big_pts, 'Economy', false);   
        big_dir =   big_dir(:,1); big_dir(3) = 0;
        keep_v  =   true(length(V),1);
        for cc = 1:length(V)
            obj_pts     =   catenary_pts(CC == V(cc), :);
            Ds      =   point_to_line_distance(obj_pts, mean(big_pts,1),...
                        mean(big_pts,1) +  big_dir' * 1);
            min_d   =   min(Ds);
            if min_d > 0.4
                keep_v(cc)  =   false;
            end
        end
        catenary_ids  	=       catenary_ids(ismember(CC,V(keep_v)));   

        % CONTACT WIRE
        contact_pts     =    cat_vx.Location(contact_ids,:);

        CC  	=   region_growing_cluster(contact_pts, 0.3);
        [C,V]   =   groupcounts(CC);
        big_c   = 	V(C > max(C)*0.8);
        big_c   =   ismember(CC, big_c);
        big_pts =   catenary_pts(big_c , :);
        big_dir =   pca(big_pts, 'Economy', false);   
        big_dir =   big_dir(:,1);   big_dir(3) = 0;
        keep_v  =   true(length(V),1);
        for cc = 1:length(V)
            obj_pts	=   catenary_pts(CC == V(cc), :);
            Ds      =   point_to_line_distance(obj_pts, mean(big_pts,1),...
                        mean(big_pts,1) +  big_dir' * 1);
            min_d   =   min(Ds);
            if min_d > 0.4
                keep_v(cc)  =   false;
            end
        end

        contact_ids  	=   contact_ids(ismember(CC,V(keep_v)));

        CATENARY_ID{i}	=    catenary_ids;
        CONTACT_ID{i}  	=    contact_ids;  

    end

    CATENARY_ID     =   cat(1, CATENARY_ID{:});
    CONTACT_ID      =   cat(1, CONTACT_ID{:});
    DROPPERS_ID 	=   cat(1, DROPPERS_ID{:});

    % cat_vx_L        =   cat_vx.Location;
    % CATENARY_ID     =   remove_cross_wires(cat_vx_L, CATENARY_ID, 0.3, 'cat');
    % CONTACT_ID      =   remove_cross_wires(cat_vx_L, CONTACT_ID, 0.3, 'con');
    % DROPPERS_ID     =   remove_cross_wires(cat_vx_L, DROPPERS_ID, 0.3, 'dro');


    % RECOVER PARENT POINTS
    CATENARY_ID     =   cat_vx.parent_pt_idx(CATENARY_ID);
    CONTACT_ID      =   cat_vx.parent_pt_idx(CONTACT_ID);
    DROPPERS_ID     =   cat_vx.parent_pt_idx(DROPPERS_ID);

    CATENARY_ID     =   all_cat_ids(cat(1, CATENARY_ID{:}));
    CONTACT_ID      =   all_cat_ids(cat(1, CONTACT_ID{:}));
    DROPPERS_ID 	=   all_cat_ids(cat(1, DROPPERS_ID{:}));



    % %% Assign remaining points
    % indices = zeros(1, length(all_ids));
    % indices(ismember(all_ids, RETURN_ID))      =   1;
    % indices(ismember(all_ids, FEED_ID))        =   2;
    % indices(ismember(all_ids, CATENARY_ID))    =   3;
    % indices(ismember(all_ids, CONTACT_ID))     =   4;
    % indices(ismember(all_ids, DROPPERS_ID))    =   5;
    % 
    % indices     =   assign_rem_pts(cloud_loc, all_ids, indices);
    % 
    % return_wire     =   all_ids(indices == 1);
    % feed_wire       =   all_ids(indices == 2);
    % catenary        =   all_ids(indices == 3);
    % contact_wire    =   all_ids(indices == 4);
    % droppers        =   all_ids(indices == 5);


    % %% Remove crossing wires
    % return_wire     =   remove_crossing_wires(cloud_loc, RETURN_IDS, 0.1, 'ret');
    % if ~isempty(FEED_IDS)
    %     feed_wire       =   remove_crossing_wires(cloud_loc, FEED_IDS, 0.1, 'fee');
    % end
    % catenary        =   remove_crossing_wires(cloud_loc, catenary, 0.1, 'cat');
    % contact_wire    =   remove_crossing_wires(cloud_loc, contact_wire, 0.1, 'con');
    % droppers        =   remove_crossing_wires(cloud_loc, droppers, 0.2, 'dro');

    %% Store output
    cabling.return_wire     =   RETURN_ID;
    cabling.feed_wire       =   FEED_ID;
    cabling.catenary        =   CATENARY_ID;
    cabling.contact_wire    =   CONTACT_ID;
    cabling.droppers        =   DROPPERS_ID;


%     %Visualise output
%     figure; hold on
% 
%     pcshow(cloud.Location(cabling.return_wire,:), 'w', 'markersize', 30)
%     pcshow(cloud.Location(cabling.feed_wire,:), 'y', 'markersize', 30)
%     pcshow(cloud.Location(cabling.catenary,:), 'b', 'markersize', 30)
%     pcshow(cloud.Location(cabling.contact_wire,:), 'c', 'markersize', 30)
%     pcshow(cloud.Location(cabling.droppers,:), 'r', 'markersize', 30)


    end


%% AUXILIAR FUNCTIONS
function [ids, ids_clust] = remove_noise(cloud_loc, ids, r, c_limit)
    ids_clust   =   region_growing_cluster(cloud_loc(ids,:), r);
    [C,V]       =   groupcounts(ids_clust); 
    keep_v      =   V(C > c_limit); 
    keep_v_ids    =   ismember(ids_clust,keep_v);

    ids         =   ids(keep_v_ids);
    ids_clust   =   ids_clust(keep_v_ids);
end

function [ids, ids_clust]	=   remove_cross_wires(vx_loc, ids, r, wire)

    pts     =   vx_loc(ids,:);
    clusts  =   region_growing_cluster(pts, r);
    num_c   =   unique(clusts);
    if length(num_c) == 1; return; end   

    [C,V]       =   groupcounts(clusts); 


% figure; 
% pcshow(pts)

    % Keep the most paralell to x
    keep_i      =   [];
    keep_ids    =   [];
    j           =   0;
    ids_clust   =   [];
    for i = 1:length(num_c)
        pts_i_id	=   find(clusts==i);
        pts_i_loc  	=   pts(pts_i_id,:);


%         hold on; pcshow(pts_i_loc,  'r','markersize', 100)

        range_pts   =   range(pts_i_loc,1);
        [pca_pts,~,lam]     =   pca(pts_i_loc, 'Economy', false);


        a1d	=   (sqrt(lam(1)) - sqrt(lam(2))) / sqrt(lam(1));
%         a2d =   (sqrt(lam(2)) - sqrt(lam(3))) / sqrt(lam(1));
%         a3d =   sqrt(lam(3)) / sqrt(lam(1));

        % All wires but catenary are in the XZ plane, thus the second ppal 
        % comp is Z
        if wire == 'cat'
            cond_1  =   pca_pts(1,1) > 0.98;% & a1d > 0.95;
            cond_2  =   a1d > 0.95; %length(pts_i_loc) > 20;
            cond_3  =   range_pts(1) > range_pts(3) * 2  & range_pts(2) < 1;
        elseif wire == 'dro'
%             cond_1 = (pca_pts(3,2) > 0.85 && pca_pts(1,1) > 0.85) || (pca_pts(3,1) > 0.85 && pca_pts(1,2) > 0.85);
%             cond_1 = (pca_pts(1,1) > 0.85 | pca_pts(1,2) > 0.85)  && a1d > 0.95;

                cond_1  =   pca_pts(3,1) > 0.8 &&  a1d > 0.8;
                cond_2  =   a1d > 0.8;
                cond_3  =   range_pts(3) > range_pts(1) * 2;
        elseif wire == 'ret'
                cond_1  =	pca_pts(1,1) > 0.9;
                cond_2  =   a1d > 0.95;
%                 cond_3  =   range_pts(1) > range_pts(3) * 2 & range_pts(2) < 1;
                cond_3  =   range_pts(2)  < 0.4;        
        elseif wire == 'fee'
                cond_1  =	pca_pts(1,1) > 0.9;
                cond_2  =   a1d > 0.95;
%                 cond_3  =   range_pts(1) > range_pts(3) * 2 & range_pts(2) < 1;
                cond_3  =   range_pts(2)  < 0.4;
        elseif wire == 'con'
            cond_1  =   pca_pts(1,1) > 0.9;% & a1d > 0.95;
            cond_2  =   a1d > 0.95; %length(pts_i_loc) > 20;
            cond_3  =   range_pts(1) > range_pts(3) * 2  & range_pts(2) < 1;
        end



        if cond_1 && cond_2 && cond_3
            j = j+1;
            keep_i      =   [keep_i, i];
            ids_clust   =   [ids_clust; j * ones(length(pts_i_id),1)];
            keep_ids    =   [keep_ids; pts_i_id];
        end
    end
    ids     =   ids(keep_ids);
end

function [idx]     =   assign_rem_pts(cloud_loc, all_ids, idx)  % NOT OK. TAKES TOO LONG
    all_loc     =   cloud_loc(all_ids,:);  
    not_as_i    =   (idx == 0); 

    not_assign  =   all_loc(not_as_i,:); 
%     assign      =   all_loc(~not_as_i,:);   
%     assign_idx  =   idx(~not_as_i);
    [n, D]      =   knnsearch(all_loc, not_assign, 'K', 30);

    indices_n   =   idx(n);
%     idx_n               =   assign_idx(n);
%     idx_n(idx_n == 0)   =   nan;

    idx_n               =   idx(n(:,2));
    idx_n(D(:,2)> 0.05)  =   10;
    idx_n_prev          =   idx_n;
    idx(not_as_i)       =   idx_n;

    while any(idx_n == 0)

        idx_n_prev      =   idx_n;
        idx_n        	=   idx(not_as_i);
        if sum(idx_n_prev - idx_n) == 0
            first_0  =   find(idx_n==0, ceil(length(idx_n)*.1));
            for j = 1:length(first_0)
                idx(not_as_i)   =   idx_n;
                indices_n       =   idx(n);
                non_zero        =   find(indices_n(first_0(j),:),1);
                if isempty(non_zero)
                    idx_n(first_0(j))	=	10;
                else
                    idx_n(first_0(j))	=	indices_n(first_0(j),non_zero);
                end                
            end
        end
        idx(not_as_i)   =   idx_n;
    end

    idx_n(D(:,2)> 0.1)  =   0;
    idx(not_as_i)   =   idx_n;

end 
























