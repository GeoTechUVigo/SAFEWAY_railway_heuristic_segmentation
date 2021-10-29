function [components] = TrackSegmentation(vx, grid, subSectionsWidth, distWall, traj)
% Segmentation of the cloud in track and notTrack, wall, wall2 and roof.
% This segmentation helps the extraction of components.
% 
% First, the cloud is voxelized in voxels big enough to have a good
% extraction.
% 
% Track extraction. If voxels are big enough, most part of them in track
% do not have neither voxels under nor up (neighbours number 5 and 22).
% Those voxels are selected and clustered. Clusters with few voxels are
% deleted. All clusters with a mean Z lower than the mean Z of the 
% trajectory are classified as ground. All voxels of those clusters are
% classified as ground  voxels. All voxels with at least 1 neighbour from 
% the ground in their horizontal plane (neighbours 10:17) are ground too.
% 
% Wall extraction. It is done 2 extractions: wall and wall2. The cloud
% is splited in sections in X axis with a with equal to subSectionsWidth.
% In each section, all voxels with a Y lower than the min track Y in the
% section and higher than max track Y are classified as a wall. Wall2 is
% the same but considering a margin equal to distWall.
% 
% These both extraction are done in a cloud with a big voxel's grid. the
% indices of ground and wall are calculated in the input cloud.
% 
% Roof extraction. A local pca analysis is applied to the cloud with voxels
% that have not been classified. Voxels with a low 3º eigenvalue (planar)
% and vertical normal (3º eigenvector) are selected and clustered.
% Clusters with few voxels are removed. All voxeles in the selected
% clusters are classified as roof.
% 
% ------------------------------------------------------------------------
% INPUTS:
% 
% vx : Voxels. Cloud 
% 
% grid : numeric. Grid size to track and wall extraction.
% 
% subSectionsWidth : numeric. Width of sections to wall extraction.
% 
% distWall : numeric. Margin between the end of the ground and the start of
%            wall2.
% 
% traj : trajectory. Trajectory.
% 
% -------------------------------------------------------------------------
% OUTPUTS:
% 
% components.track : track indexes.
% 
% components.notTrack : indexes of the cloud that are niether in track not  
%                         in roof
% 
% components.wall : wall indexes
% 
% components.wall2 : wall indexes with a margin
% 
% components.roof : roof indexes
%
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 22/12/2020

%% Voxeling bigger to avoid problems with small elements that are from ground
vxBigVoxels  = Voxels(vx,grid);
restElements = find(1:1:length(vxBigVoxels.Location))';

%% Track
%Only voxels whitout neighbours just above or just below itselft
noUpDownIdx = find((isnan(vxBigVoxels.neighbours(:,22)) | isnan(vxBigVoxels.neighbours(:,5))) & vxBigVoxels.Location(:,3) < mean(traj.points(:,3)));

% figure; pcshow(vxBigVoxels.Location,'b','MarkerSize',200);
% hold on; pcshow(vxBigVoxels.Location(noUpDownIdx,:),'g','MarkerSize',200);

% Clustering the selected voxels
vxGround        = select(vxBigVoxels,noUpDownIdx);
idx             = ClusteringNeighbours(vxGround,'distance',1.2*grid);
[num]           = groupcounts(idx);
[num, clusters] = sort(num,'descend'); % Sortin the clusters by num of voxels
clusters        = clusters(num > 3/vx.grid); % antes 300 voxeles
% Choosing the biggest voxels with the mean height less than the mean
clustersGround = false(size(clusters));
maxHeight      = mean(traj.points(:,3),1);
for i =1:numel(clusters)
    
    if mean(vxBigVoxels.Location(noUpDownIdx(idx == clusters(i)),3),1) < maxHeight  % Ground
        
%         maxHeight = mean(vxBigVoxels.Location(realGround,3));        
        clustersGround(i) = true;
%         break;
        
    end
end

track         = noUpDownIdx(ismember(idx,clusters(clustersGround)));
restElements  = restElements(~ismember(restElements,track));

% Voxels with at least 1 neighbour in xy plane from ground are ground too
xyNeighboursrestElements = vxBigVoxels.neighbours_rows(restElements,(10:17)); % neighbours in xy plane of not ground voxels
groundToo                = sum(ismember(xyNeighboursrestElements, track),2) > 0; % > 0 if at least 1 is from the ground 

track         = [track; restElements(groundToo)];
restElements  = restElements(~ismember(restElements,track));

% figure; pcshow(vxBigVoxels.Location(track,:),'y','MarkerSize',200);
% hold on; pcshow(vxBigVoxels.Location(restElements,:),'b','MarkerSize',200);
% hold on; pcshow(vxBigVoxels.Location(noUpDownIdx(idx == clusters(1)),:),'r','MarkerSize',200);

%% Wall
% Making small sections by x direction.

maxX = max(vxBigVoxels.Location(:,1));
minX = min(vxBigVoxels.Location(:,1));
    
% figure; pcshow(vxBigVoxels.Location,'b', 'MarkerSize',100);

wall = false(size(vxBigVoxels.Location,1),1); % wall voxels
wall2 = false(size(vxBigVoxels.Location,1),1); % wall voxels but with a margin to not cut elements

for j = minX:vxBigVoxels.grid:maxX
    minSec = j;
    maxSec =  minSec + subSectionsWidth;
    
    sectionIds    = find(vxBigVoxels.Location(:,1) >= minSec & vxBigVoxels.Location(:,1) < maxSec);
    realGroundSec = sectionIds(ismember(sectionIds, track));
        
    limitWallRight = max(vxBigVoxels.Location(realGroundSec,2)) - vxBigVoxels.grid;
    limitWallLeft  = min(vxBigVoxels.Location(realGroundSec,2)) + vxBigVoxels.grid;
    
    if ~isempty(limitWallRight) && ~isempty(limitWallLeft)
        % Right wall
        thisWallR = vxBigVoxels.Location(sectionIds,2) >= limitWallRight; % right idx       
        thisWallR = sectionIds(thisWallR);
        
        % Left wall
        thisWallL = vxBigVoxels.Location(sectionIds,2) <= limitWallLeft; % points between limits
        thisWallL = sectionIds(thisWallL);
        
        % Both walls
        thisWall = [thisWallR;thisWallL];
        wall(thisWall(ismember(thisWall,restElements))) = true;
        
        % Same but with less restriction to not cut cables
        % Right wall
        thisWallR = vxBigVoxels.Location(sectionIds,2) >= limitWallRight + distWall; % right idx       
        thisWallR = sectionIds(thisWallR);
        
        % Left wall
        thisWallL = vxBigVoxels.Location(sectionIds,2) <= limitWallLeft - distWall; % points between limits
        thisWallL = sectionIds(thisWallL);
        
        thisWall = [thisWallR;thisWallL];
        wall2(thisWall(ismember(thisWall,restElements))) = true;
    end
end

wall = find(wall);
wall2 = find(wall2);

restElements = restElements(~ismember(restElements,wall));

% figure; pcshow(vxBigVoxels.Location(track,:),'y','MarkerSize',200);
% hold on; pcshow(vxBigVoxels.Location(restElements,:),'g','MarkerSize',200);
% hold on; pcshow(vxBigVoxels.Location(wall,:),'r','MarkerSize',200);
% hold on; pcshow(vxBigVoxels.Location(wall2,:),'g','MarkerSize',200);

%% Going back to vx idx
track     = vxBigVoxels.parent_idx(track,:);
wall      = vxBigVoxels.parent_idx(wall,:);
wall2     = vxBigVoxels.parent_idx(wall2,:);
restElements = vxBigVoxels.parent_idx(restElements,:);

track     = cat(1,track{:});
wall      = cat(1,wall{:});
wall2     = cat(1,wall2{:});
restElements = cat(1,restElements{:});

%% Roof
% Pca Local analysis in restElements
vxSelected = select(vx, restElements);
directions = PcaLocal(vxSelected,3*vxSelected.grid,1);
% normals = pcnormals(pointCloud(vxBigVoxels.Location),100);

points          = abs(directions.eigenvectors(:,9)) > 0.7 & directions.eigenvalues(:,1) < 0.7; % vertical normal and a planar element
points          = restElements(points);
vxSelected      = select(vx, points);
idx             = ClusteringNeighbours(vxSelected,'distance',1.3*grid);
[num]           = groupcounts(idx);
[num, clusters] = sort(num,'descend'); % Sortin the clusters by num of voxels
clusters = clusters(num > 100/vx.grid); 
roof = points(ismember(idx, clusters));

% figure; pcshow(vx.Location, 'b', 'MarkerSize', 50)
% hold on; pcshow(vxBigVoxels.Location(track,:), 'r', 'MarkerSize', 50)
% hold on; pcshow(vx.Location(roof,:), 'y', 'MarkerSize', 50)

%% Saving element

% notTrackRough has more voxels than notTrack to not cut any element
% It will be used one ore other depends on if the element to extrack is
% close or not to a wall
notTrack = find(1:1:length(vx.Location))';
notTrack = notTrack(~ismember(notTrack, [track; roof]));

% figure; pcshow(vx.Location,'b','MarkerSize',50);
% hold on; pcshow(vx.Location(track,:),'g','MarkerSize',50);
% hold on; pcshow(vx.Location(roof,:),'w','MarkerSize',50);
% hold on; pcshow(vx.Location(wall,:),'y','MarkerSize',50);
% hold on; pcshow(vx.Location(wall2,:),'r','MarkerSize',50);

components.track           = track;
components.notTrack        = notTrack;
components.wall            = wall;
components.wall2           = wall2;
components.roof            = roof;
end
