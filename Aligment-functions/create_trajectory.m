function traj = create_trajectory(cloud)
% Function that creates a trajectory based on a point cloud angle data 
% Inputs: 
%  - cloud: pointCloud_ object
% Outputs: 
%  - traj: trajectory object
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Select the points with zero angle, which means that are aligned to 
    % the scan direction
%     dir_pts_ids     =   find(abs(cloud.angle)< 0.5);
    dir_pts_ids     =   find(cloud.angle == 19);%,cloud.angle < 19));

    % Voxelise the chosen points and perform a region growing algorithm to
    % deprecate the points above the ground
    dir_cloud       =   select(cloud, dir_pts_ids);
    vx_dir_cloud    =   Voxels(dir_cloud, 0.2);

    clust_ids       =   region_growing_cluster(vx_dir_cloud.Location, 0.5);
    [C,V]           =   groupcounts(clust_ids);
    traj_gr_ids     =   V(C == max(C));
    traj_gr_ids     =   clust_ids == traj_gr_ids;

    parent_ids      =   vx_dir_cloud.parent_pt_idx(traj_gr_ids,:);
    parent_ids      =   dir_pts_ids(cat(1,parent_ids{:}));

    % Voxelise again the selected points to speed up the next computations
    traj_cloud      =   select(cloud, parent_ids);
    vx_traj_cloud   =   Voxels(traj_cloud, 1);


    %%
    rem_ids     =   true(1,length(vx_traj_cloud.Location));
    traj_ids    =   false(1,length(vx_traj_cloud.Location));
    while sum(rem_ids) ~= 0 
        rem_pts         =   vx_traj_cloud.Location(rem_ids,:); 
        first_id        =   find(rem_ids,1);
        first_pt        =   vx_traj_cloud.Location(first_id,:);
        distances       =   pdist2(rem_pts, first_pt);
        rem_id_find     =   find(rem_ids);
        select_ids      =   rem_id_find(distances <= 5);

        % Update the visited points and store the traj id
        traj_ids(first_id)  =   true;
        rem_ids(select_ids) =   false;
    end
    % Find the closest points in the original cloud to the defined traj points
    % and store their time stamp. The actual traj points are elevated 1.5 m.
    traj_pts_ids    =   knnsearch(cloud.Location, ...
                            vx_traj_cloud.Location(traj_ids,:));
    traj_pts        =   cloud.Location(traj_pts_ids,:);
    traj_pts(:,3)   =   traj_pts(:,3) + 1.5;
    traj_time       =   cloud.timeStamp(traj_pts_ids,:);

    [traj_time, sort_i]     =   sort(traj_time);
    traj_pts                =   traj_pts(sort_i, :);

    % Create the trajectory object
    traj            =   trajectory(traj_pts, traj_time);
end

