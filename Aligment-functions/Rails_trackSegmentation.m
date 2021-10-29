function [track_id, notTrack_id] = Rails_trackSegmentation(varargin)
% Function that segmentates the ground of a point cloud using a region
% growing algorithm. The seeds used fot the process are selected as the
% closest points to each of the trajectory points.
%
%Inputs: 
% - vx: Nube de puntos voxelizada (objeto clase Voxels)
% - traj: Objeto de clase trajectory
%
%Outputs: 
%  - track_id: indices of the points classified as track.
%  - notTrack_id: indices of the points not classified as track.
% 
%%
    if (numel(varargin) == 2)
        VX      =   varargin{1}; %Estructura voxel
        traj    =   varargin{2}; %Trayectoria
    else
        fprintf('Wrong number of inputs... \n')
    end

    % Center the cloud and trajectory the cloud centre
    VX_pts      =   VX.Location;
    traj_pts    =   traj.points;
    traj_pts    =   traj_pts - mean(VX_pts);
    VX_pts      =   VX_pts - mean(VX_pts);
    
    if ~isempty(traj.pitch)
        rot_x       =   mean(traj.pitch);
        rot_x       =   rotx(rot_x);

        traj_pts    =   traj_pts * rot_x;
        VX_pts      =   VX_pts * rot_x;

        angle_rot   =   mean(traj.pitch);
        VX_pts      =   (rotx(angle_rot) * (VX_pts'))';
        traj_pts    =   (rotx(angle_rot) * (traj_pts'))';
    end
    % Perform a rangesearch in the cloud and compute the mean height and 
    % the heights standard deviation in each neibourhood.
    RS      =   rangesearch(VX_pts, VX_pts, 0.8, 'SortIndices',false);
    RS_h    =   cellfun(@(x) VX_pts(x,3), RS,  'UniformOutput' , false);
    RS_std  =   cellfun(@(x) std(x), RS_h);
    RS_avg  =   cellfun(@(x) mean(x), RS_h);

    % Define a limit for the standard deviation and average heigh
    lim_std = 0.25;
    lim_select      =   RS_std < lim_std & RS_avg < mean(traj_pts(:,3))+1;
    ground_ids      =   find(lim_select);
%     
%     figure; pcshow(VX_pts(ground_ids,:))

    %%%%%%%%%%%%%% OLD APPROACH
%     selection       =   VX.Location(lim_select,:);
%     vx_cluster_ids  =   region_growing_cluster(pointCloud_(selection), 0.6); 
%     cluster_points  =   histcounts(vx_cluster_ids, [unique(vx_cluster_ids); max(vx_cluster_ids)+1]);
% %     big_c           =   find(cluster_points > 300);
%     [~, big_c]      =   max(cluster_points);
%     members         =   ismember(vx_cluster_ids, big_c);
    %%%%%%%%%%%%%% OLD APPROACH
    
     %%%%%%%%%%%%%% NEW APPROACH
%     ground_ids         =   find(VX_pts(:,3) < max(traj_pts(:,3)));
	[N, edges, bin]     =   histcounts(VX_pts(ground_ids,3), 'BinWidth', 0.50);
    [max_N, max_bin]    =   max(N); 
    edges               =   edges(1:end-1); 
    try
        max_edge            =   edges(max_bin + 4);
    catch
        max_edge            =   edges(end);
    end
    try
        min_edge            =   edges(max_bin - 3);
    catch
        min_edge =  edges(1);
    end
    limit_bins      =   find(N > max_N * 0.1 & edges <= max_edge & edges >= min_edge);
    members         =   (ismember(bin,limit_bins)); 
%     figure; histogram(VX_pts(ground_ids,3), 'BinWidth', 0.50)
    %%%%%%%%%%%%%% NEW APPROACH

    real_ground     =   ground_ids(members);
    not_ground      =   [ground_ids(~members); find(~lim_select)];

    track_id       =   VX.parent_pt_idx(real_ground,:);
    notTrack_id    =   VX.parent_pt_idx(not_ground,:);

    track_id       =   cat(1,track_id{:});
    notTrack_id    =   cat(1,notTrack_id{:});
end









