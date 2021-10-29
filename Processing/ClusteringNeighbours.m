function [clusterIndex] = ClusteringNeighbours(varargin)
 % Making clusters of voxels. Voxels that are neighbour of any
 % voxel of the cluster and meets the requirements specified in the
 % arguments are added to the cluster.
distance = 'distance';
maxZ = 'max_z';
direction = 'direction';
seeds = 'seeds';

if (numel(varargin) == 1)
    vx = varargin{1}; % Cloud of voxels
    neighbors = vx.neighbours_rows;

elseif (strcmp(varargin{2},seeds)) % Considering seeds
    vx = varargin{1}; % Cloud of voxels
    neighbors = vx.neighbours_rows;
    seeds = varargin{3};

elseif (strcmp(varargin{2},distance)) % Considering distance
    vx = varargin{1};
    distance = varargin{3};
    neighbors = vx.neighbours_rows;
    location = vx.Location; 

elseif (strcmp(varargin{2},maxZ)) % Considering max_z
    vx = varargin{1};
    maxZ = varargin{3};
    neighbors = vx.neighbours_rows;
    vxMeanZ = vx.mean_z;
    vxMaxZ = vx.max_z;
    seeds = varargin{4};
    
elseif (strcmp(varargin{2},direction)) % Considering direction
    vx = varargin{1};
    max_variation = varargin{3};
    neighbors = vx.neighbours_rows;
    location = vx.Location;
    direction = varargin{4};
end 



clusterIndex = zeros(size(vx.index)); % vector that will have the idx of the cluster of each voxel
nCluster = 1; % first idx cluster
clusterIndex(1) = nCluster;
visited = false(numel(vx.index),1); % vector to save the voxels visited
%% All the neighbours are from the same cluster
if (numel(varargin) == 1)

notAllPoints = true;
while notAllPoints % while all the points are not visited
    notAllPoints = false;
    
    propagationZone = true;
    while propagationZone % while the cluster is growing
        propagationZone = false;

        propagation = and(clusterIndex == nCluster, ~visited); % new voxels of this cluster

        neighborsPropagation = neighbors(propagation,:);
        neighborsPropagation = cat(1,neighborsPropagation);
        neighborsPropagation = neighborsPropagation(~isnan(neighborsPropagation));
        neighborsPropagation = unique(neighborsPropagation); % neighbours of the new voxels
        if (numel(neighborsPropagation)>0) % if there are neighbours
            propagationZone = true; % the cluster has growed
            clusterIndex(neighborsPropagation) = nCluster; % saving the cluster idx of the new voxels
        end

        visited(propagation) = 1; % saving voxels visited
    end
    
    if (~all(visited)) % if not all voxels have been visited
       notAllPoints = true; 
       nCluster = nCluster +1; % next cluster idx
       clusterIndex(find(visited==0, 1, 'first')) = nCluster; % next voxel with the new cluster idx
    end
    

end

%% Only seeds make clusters. All the neighbours are from the same cluster
elseif (~isa(seeds,'char') == true)
if ~isa(seeds,'logical')
   seeds2 = false(length(vx.Location),1) ;
   seeds2(seeds) = true;
   seeds = seeds2;
end
clusterIndex = zeros(size(vx.index)); % vector that will have the idx of the cluster of each voxel
nCluster = 1; % first idx cluster
visited = false(numel(vx.index),1); % vector to save the voxels visited
clusterIndex(find(visited==0 & seeds, 1, 'first')) = nCluster;
notAllPoints = true;

while notAllPoints % while all the points are not visited
    notAllPoints = false;
    
    propagationZone = true;
    while propagationZone % while the cluster is growing
        propagationZone = false;

        propagation = and(clusterIndex == nCluster, ~visited); % new voxels of this cluster

        neighborsPropagation = neighbors(propagation,:);
        neighborsPropagation = cat(1,neighborsPropagation);
        neighborsPropagation = neighborsPropagation(~isnan(neighborsPropagation));
        neighborsPropagation = unique(neighborsPropagation); % neighbours of the new voxels
        if (numel(neighborsPropagation)>0) % if there are neighbours
            propagationZone = true; % the cluster has growed
            clusterIndex(neighborsPropagation) = nCluster; % saving the cluster idx of the new voxels
        end

        visited(propagation) = 1; % saving voxels visited
    end
    
    if (~all(visited(seeds))) % if not all seeds have been visited
       notAllPoints = true; 
       nCluster = nCluster +1; % next cluster idx
       clusterIndex(find(visited==0 & seeds, 1, 'first')) = nCluster;
    end
    

end

%% Considering distance
elseif (~isa(distance,'char') == true)
    
notAllPoints = true;
while notAllPoints % while all the points are not visited
    notAllPoints = false;
    
    propagationZone = true;
    while propagationZone % while the cluster is growing
        propagationZone = false;

        propagation = find(and(clusterIndex == nCluster, ~visited)); % new voxels of this cluster
        
        for i = 1: numel(propagation) % going through all the voxels in propagation
            voxel = location(propagation(i),:); % location of the voxel
            neighboursVoxel = neighbors(propagation(i),:); 
            neighboursVoxel = neighboursVoxel(~isnan(neighboursVoxel)); % neighbours of the voxel
            neighboursVoxelLocation = location(neighboursVoxel,:);
            neighboursVoxelLocation = neighboursVoxelLocation - voxel; % distance between the neighbours and voxel in xyz
            for j = 1:numel(neighboursVoxel) % going through all the neighbours of this voxel
                if (norm(neighboursVoxelLocation(j,:)) < distance) % if it is close enough it is from the same cluster
                    propagationZone = true; % the cluster has growed
                    clusterIndex(neighboursVoxel(j)) = nCluster; % saving the cluster idx of the new voxels
                end
            end            
        end
        
        visited(propagation) = 1; % saving voxels visited
    end
    
    if (~all(visited)) % % if not all voxels have been visited
       notAllPoints = true;
       nCluster = nCluster +1; % next cluster idx
       clusterIndex(find(visited==0, 1, 'first')) = nCluster; % next voxel with the new cluster idx
    end
    
end

%% Considering max_z
elseif (~isa(maxZ,'char') == true)

% Modifing cloud pca to flattenit in the xy plane in order to filter the points by height (z)
vxLocationOriented = vx.Location;
[vxPca,~,~] = pca(vxLocationOriented, 'Economy', false);
vxPcaFixed = zeros(size(vxPca));
vxPcaFixed(:,1) = vxPca(:,1); % principal eigenvector
vxPcaFixed(:,2) = [-vxPca(2,1), vxPca(1,1), 0] / norm([-vxPca(2,1), vxPca(1,1), 0]); % orthonormal to pca1 and in the xy plane
rot90Pca2= makehgtform('axisrotate',vxPcaFixed(:,2),90*pi/180); % rotate 90 degrees in pca2 4x4 matrix
rot90Pca2 = rot90Pca2(1:3,1:3); % 3x3 matrix
vxPcaFixed(:,3) = vxPcaFixed(:,1)' * rot90Pca2; % orthonormal to pca1 and pca2
vxLocationOriented = vxLocationOriented * vxPcaFixed;
    
clusterIndex = zeros(size(vx.index)); % vector that will have the idx of the cluster of each voxel
nCluster = 1; % first idx cluster
clusterIndex(seeds(1)) = nCluster;
visited = false(numel(vx.index),1); % vector to save the voxels visited
propagation = false(numel(vx.index),1);
clusterPcaFixed = zeros(3,3);
notAllPoints = true;

contador = 0;

% VX_Location_oriented = vx.Location;
% figure;pcshow(vx.Location,'b','MarkerSize',50);
while notAllPoints % while all the points are not visited
    notAllPoints = false;
    
    propagation(:) = false;
    propagation(clusterIndex == nCluster) = true; % seed
    meanZCluster = mean(vxLocationOriented(propagation,3));
    propagationZone = true;
    while propagationZone % while the cluster is growing
        propagationZone = false;
       
        contador = contador +1;
        if contador == 800
            contador;
        end
         contador;       
        %PCA
        if true %range(vx.Location(cluster_index == nCluster,1)) < 1
            
            
%             mean_z_cluster = mean([mean_z_cluster;mean_z_cluster; mean(vx.mean_z(propagation))]);
            meanZCluster = mean([meanZCluster;meanZCluster; mean(vxLocationOriented(propagation,3))]);
%             mean_z_cluster = mean(vx.max_z(cluster_index == nCluster));
%             vx_location_fixed = vx.Location;
%             cluster = cluster_index == nCluster;

%             if min(vx.Location(cluster_index == nCluster,1)) - min(vx.Location(cluster,1)) <  - 2.25 | first_time
        elseif false
            cluster = (clusterIndex == nCluster) & (vx.Location(:,1) < min(vx.Location(cluster,1)) + 50);
            [cluster_pca,~,~] = pca(vx.Location(cluster,:), 'Economy', false);
            
            clusterPcaFixed(:,1) = cluster_pca(:,1); % principal eigenvector
            clusterPcaFixed(:,2) = [-cluster_pca(2,1), cluster_pca(1,1), 0] / norm([-cluster_pca(2,1), cluster_pca(1,1), 0]); % orthonormal to pca1 and in the xy plane
            rot90Pca2= makehgtform('axisrotate',clusterPcaFixed(:,2),90*pi/180); % rotate 90 degrees in pca2 4x4 matrix
            rot90Pca2 = rot90Pca2(1:3,1:3); % 3x3 matrix
            clusterPcaFixed(:,3) = clusterPcaFixed(:,1)' * rot90Pca2; % orthonormal to pca1 and pca2
        
            vxLocationOriented = cat(2,vx.Location(:,[1,2]),vx.max_z) * clusterPcaFixed;
            meanZCluster = mean(vxLocationOriented(clusterIndex == nCluster,3));
            
            
%             figure;pcshow(vx.Location,'b','MarkerSize',50);
%             hold on; pcshow(vx.Location(cluster_index == nCluster,:),'r','MarkerSize',50);
%             hold on; pcshow(vx.Location(cluster,:),'r','MarkerSize',50);
        end
       
         
%          auxiliar = cluster_index;
%          total_neighbours = neighbors(propagation,:);
%          total_neighbours = total_neighbours(~isnan(total_neighbours)); % neighbours of the voxel
        
        neighboursVoxel = unique(neighbors(propagation,:)); 
        neighboursVoxel = neighboursVoxel(~isnan(neighboursVoxel)); % neighbours of the voxel
        neighboursVoxel = neighboursVoxel(clusterIndex(neighboursVoxel) == 0);
        neighboursVoxelZ = vxLocationOriented(neighboursVoxel,3) - meanZCluster;
%         hold on; pcshow(vx.Location(neighbours_voxel,:),'y','MarkerSize',50);
        if any(neighboursVoxelZ > - maxZ)% if any is close enough it is from the same cluster
            propagationZone = true; % the cluster has growed
            propagation(:) = false;
            propagation(neighboursVoxel(neighboursVoxelZ > - maxZ)) = true; 
            clusterIndex(neighboursVoxel(neighboursVoxelZ > - maxZ)) = nCluster; % saving the cluster idx of the new voxels
        end
        
        
      
%         for i = 1: numel(propagation) % going through all the voxels in propagation
%             %voxel = vx_max_z(propagation(i),:); % location of the voxel
%             neighbours_voxel = neighbors(propagation(i),:); 
%             neighbours_voxel = neighbours_voxel(~isnan(neighbours_voxel)); % neighbours of the voxel
%             neighbours_voxel_z = vx_location_fixed(neighbours_voxel,3) - mean_z_cluster;
%             if any(neighbours_voxel_z > - max_z)% if any is close enough it is from the same cluster
%                 propagation_zone = true; % the cluster has growed
%                 cluster_index(neighbours_voxel(neighbours_voxel_z > - max_z)) = nCluster; % saving the cluster idx of the new voxels
%                 propagation = cluster_index(neighbours_voxel(neighbours_voxel_z > - max_z));
%             end
% %             for j = 1:numel(neighbours_voxel) % going through all the neighbours of this voxel
% %                 if (((neighbours_voxel_z(j)) > - max_z)) % if it is close enough it is from the same cluster
% %                     propagation_zone = true; % the cluster has growed
% %                     cluster_index(neighbours_voxel(j)) = nCluster; % saving the cluster idx of the new voxels
% %                 end
% %             end            
%         end
%         total_neighbours = unique(total_neighbours(total_neighbours ~= 0));
%         hold on;pcshow(vx_location_fixed(total_neighbours,:),'g','MarkerSize',100);
%         hold on;pcshow(vx_location_fixed(cluster_index == nCluster,:),'y','MarkerSize',100);
%         hold on;pcshow(vx_location_fixed(auxiliar == nCluster,:),'r','MarkerSize',100);

%         visited(propagation) = true; % saving voxels visited
    end
    
    if (any(clusterIndex(seeds) == 0)) % % if not all seeds have been visited
%        hold on; pcshow(vx.Location(cluster_index == nCluster,:),'g','MarkerSize',50);
       notAllPoints = true;
       nCluster = nCluster +1; % next cluster idx
       clusterIndex(seeds(find(clusterIndex(seeds)==0, 1, 'first'))) = nCluster; % next voxel with the new cluster idx
    end
    
end

%% Considering direction
elseif (~isa(direction,'char') == true)
    
notAllPoints = true;
while notAllPoints % while all the points are not visited
    notAllPoints = false;
    
    propagationZone = true;
    while propagationZone % while the cluster is growing
        propagationZone = false;

        propagation = find(and(clusterIndex == nCluster, ~visited)); % new voxels of this cluster
        
        for i = 1: numel(propagation) % going through all the voxels in propagation
            voxel = direction(propagation(i),:); % direction of the voxel
            neighboursVoxel = neighbors(propagation(i),:); 
            neighboursVoxel = neighboursVoxel(~isnan(neighboursVoxel)); % neighbours of the voxel
            neighboursVoxelLocation = direction(neighboursVoxel,:);
            neighboursVoxelLocation = neighboursVoxelLocation - voxel; % distance between the neighbours and voxel direction
            for j = 1:numel(neighboursVoxel) % going through all the neighbours of this voxel
                if (norm(neighboursVoxelLocation(j,:)) < max_variation) % if it is close enough it is from the same cluster
                    propagationZone = true; % the cluster has growed
                    clusterIndex(neighboursVoxel(j)) = nCluster; % saving the cluster idx of the new voxels
                end
            end            
        end
        
        visited(propagation) = 1; % saving voxels visited
    end
    
    if (~all(visited)) % % if not all voxels have been visited
       notAllPoints = true;
       nCluster = nCluster +1; % next cluster idx
       clusterIndex(find(visited==0, 1, 'first')) = nCluster; % next voxel with the new cluster idx
    end
    
end
end
end

