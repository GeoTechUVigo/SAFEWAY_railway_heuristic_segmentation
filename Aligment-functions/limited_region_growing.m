function [cluster_idx] = limited_region_growing(varargin)

    cloud_loc   = varargin{1};
    if ~isa(cloud_loc, 'double')
        cloud_loc   =   cloud_loc.Location;
    end
    neighbourhood   =   varargin{2};
    seeds           =   varargin{3};
        
    % Perform a range search within a specified radius
    range_indices       =   rangesearch(cloud_loc, cloud_loc, neighbourhood); 
    % Define the cluster idx for each point using the seeds as different
    % clusters
    cluster_idx     =   zeros(size(cloud_loc,1),1); 
	seed_loc        =   ismember(cloud_loc, seeds, 'rows');    
    cluster_idx(seed_loc)     =   1;
    
    cluster_idx_prev    =   cluster_idx;
    
%     min_sum     =   length(cluster_idx);
    nCluster    =   1;
%     while sum(logical(cluster_idx)) < min_sum
%         nCluster    =   nCluster + 1;
        propagate   =   true;
        while (propagate)
            propagate   =   false;
            clustered_vxs    =   find(cluster_idx == nCluster);
            if numel(clustered_vxs) == 0  
                clustered_vxs    =   find(cluster_idx == 0, 1);
                cluster_idx(clustered_vxs)   =   nCluster;
            end
            grow_indices    =   range_indices(clustered_vxs);
            % Filter by the change in height
           clustered_vxs_C  =   mat2cell(clustered_vxs,ones(1,length(clustered_vxs)));
%            var_h   =   cellfun(@(x,y) mean(cloud_loc(x,3)) - (cloud_loc(y,3)), grow_indices,clustered_vxs_C);
           var_h   =   cellfun(@(x) var(cloud_loc(x,3)), grow_indices);
            
            grow_indices    =   cat(2, grow_indices{var_h < 0.05});
            cluster_idx(grow_indices)   =   nCluster;
            if ~isequal(cluster_idx, cluster_idx_prev)
                propagate   =   true;
            end
            cluster_idx_prev   =   cluster_idx;
        end
    end

