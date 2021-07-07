function [remote] = remote_points(cloud, trajCloud, all_trajs, max_dist)
% This function is used to remove all remote point from any traj. One cloud
% is used to orient the cloud. Ones it is oriented, the mean Y of all the
% trajectories of this cloud (this cloud might have more tahn one 
% trajectory) is calculated. All the points that dist more than max_dist
% from the centre of the trajectories are removed.
% -------------------------------------------------------------------------
% INPUTS:
% cloud: PointCloud
%
% trajCloud: trajectory.
%
% all_trajs: cell of trajectory. All the trajectories.
%
% max_dist: numeric.
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 06/07/2021
%--------------------------------------------------------------------------
%% Orienting cloud and trajectories, and remove all_trajs points data are out of this cloud
pca_traj= PcaFlattering(trajCloud.points);
mean_traj = mean(trajCloud.points);

trajCloud.points = trajCloud.points * pca_traj;
cloud.Location = cloud.Location - mean_traj;
cloud.Location = cloud.Location * pca_traj;

mean_y = zeros(numel(all_trajs),1);
for i = 1:numel(all_trajs)
    all_trajs{i}.points =  all_trajs{i}.points - mean_traj;
    all_trajs{i}.points = all_trajs{i}.points * pca_traj;
    
    all_trajs{i} = select(all_trajs{i}, all_trajs{i}.points(:,1) > min(cloud.Location(:,1)) & all_trajs{i}.points(:,1) < max(cloud.Location(:,1)));
    mean_y(i) = mean(all_trajs{i}.points(:,2));
end

%% Calculing remote points
remote = cloud.Location(:,2) < (min(mean_y) - max_dist) | cloud.Location(:,2) > (max(mean_y) + max_dist);

% figure; pcshow(cloud.Location,'g');
% hold on; pcshow(cloud.Location(remote,:),'b');
% for i = 1:numel(all_trajs)
%     hold on; pcshow(all_trajs{i}.points ,'r', 'MarkerSize',200);
% end

end

