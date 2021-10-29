function [newClusters] = MergingRails(vxLocation, rails, traj)
%
% Function to merge rails from differents contact-catenary wires.
%
% Each pair of rail is compared with the other pairs, analysing their
% common points. If their common point is the last part of a rail of the
% pair i and the begining part of a rail of the pair j, the pair j is 
% continuation of i.
%
% The pairs of rails are asigned to a cluster. Each pair has the same index
% as the pair of which it is its continuation. The index of rails without 
% predecessor is equal to its position in rails.
%
% -------------------------------------------------------------------------
% INPUTS:
%
% vxLocation : Nx3 numeric. .Location of the cloud
%   
% traj : trajectory. Trajectory of vx
%
% rails : cell array with the indexes of the pair of rails in the cloud.
%         The index of each group is the position in rails of the first 
%         pair of each group. 
%
% -------------------------------------------------------------------------
% OUTPUTS:
%
% newClusters : Nx1 numeric. Indexes of the cluster of each pair of rails.
%               The index of each cluster is the position in rails of the
%               first pair of each cluster.
% 
%
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 09/04/2021

%% Orienting the cloud
vxLocation = vxLocation * PcaFlattering(traj.points);
vxLocation = vxLocation - mean(vxLocation);

% figure; pcshow(vxLocation);

%% initializing
clusterContinuation = zeros(length(rails),1);
joined = false;
for i = 1:length(rails)

    for j = 1:length(rails)
        if any(ismember(cat(1,rails{i}{:}),cat(1,rails{j}{:}))) && i ~= j % They have voxels in common
            for k = 1:2 % Left and right rail
                
%                 figure; pcshow(vxLocation(rails{i}{k},:),'g')
%                 hold on; pcshow(vxLocation(rails{j}{k},:),'r')
                
                commonVoxels = ismember(rails{i}{k},rails{j}{k});
                if any(commonVoxels)
%                     hold on; pcshow(vxLocation(rails{i}{k}(commonVoxels),:),'y')

                    % Analysing if the common part is the last part of rail{i}
                    % and the beggining part of rail{j}
                    if (pdist2(max(vxLocation(rails{i}{k}(commonVoxels,1),1)), max(vxLocation(rails{i}{k}(:,1),1))) < 1) && (pdist2(min(vxLocation(rails{i}{k}(commonVoxels,1),1)), min(vxLocation(rails{j}{k}(:,1),1))) < 1)
                        joined = true;
                        clusterContinuation(i) = j;
                        break;
                    end
                end
                
            end
            
            if joined
                joined = false;
                break;
            end
        end
    end
end

% hold on; pcshow(vxLocation(cat(1,rails{3}{:}),:),'r')
% hold on; pcshow(vxLocation(cat(1,rails{4}{:}),:),'g')

%% Calculating indexes of the clusters
% newCluster has the number of the next rail
% newCluster2 is modified, having just one number for all the rails that are
% merged
newClusters = clusterContinuation;
for i = find(clusterContinuation == 0)' % rails with no previous rail
    newClusters(i) = i;
    a = i;
    while ~isempty(find(clusterContinuation == a,1))
        a = find(clusterContinuation == a,1);
        newClusters(a) = i;
    end
end
end

