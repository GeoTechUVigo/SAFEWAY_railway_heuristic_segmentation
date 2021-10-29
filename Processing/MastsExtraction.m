function [components] = MastsExtraction(vx, components, mastNoBracketModel)
% Extraction of masts and its brackets. SE CAMBIO EN MENSULAS BORRAR WALL2
% EN VEZ DE WALL1 Y SE CAMBIÓ COMO SE COGEN LOS PUNTOS CERCANOS AL MASTIL.
% QUIZAS SE DEBA AJUSTAR MAS. BUSCAR MASTILES GRANDES
%
% Rough masts without brackert extraction. Using the result of PcaLocal
% saved in components.directionsAll, voxels with a vertil first eigenvector
% in in notTrack are selected and its neighbours added.
% These voxels are clustered.
% 
% The clusters are checked. Each mast is compared with mastNoBracketModel.
% 
% Adding points to complete the mast with all its voxels. For that, all
% voxels closer to the center XY of the cluster than the specifications of 
% the model are added. These voxels are clustered and the cluster with the 
% greatest number of original voxel is selected as mast witouth bracket.
% 
% Mast with brackets. Voxels with a low X component in its first autovector
% in notTrack and not in wall2 are selected and its neighbours too. 
% 
% These voxels are clusterd. Then, the contact between the clusters and the
% masts without brakets previously extracted is analyzed.
% 
% Masts without brackets are sorted by X
% 
% The clusters in contact with each masts without bracket are added to them
% removing repeated voxels.
% 
% -------------------------------------------------------------------------
% INPUTS:
% 
% vx : Voxels. Cloud
% 
% components : cell. Cell with elements extracted
%
% mastNoBracketModel : Element. Mast's model without bracket
% 
% -------------------------------------------------------------------------
% OUTPUTS:
% 
% components.roughMasts : cell 1 x numMasts. In each cell there are the 
%                         indexes of each mast without its bracket
% 
% components.masts :  cell 1 x numMasts. In each cell there are the 
%                     indexes of each mast with its bracket
%
%--------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 22/12/2020

%% Voxels of masts without brackets
% Mast are vertical
minAutovectorMastNoBracket = [NaN,NaN,0.7,NaN,NaN,NaN,NaN,NaN,NaN];
minAutovalueMastNoBracket  = [NaN,NaN,NaN];
maxAutovectorMastNoBracket = [0.4,0.4,NaN,NaN,NaN,NaN,NaN,NaN,NaN];
maxAutovalueMastNoBracket  = [NaN,NaN,NaN];

possibleMasts = components.notTrack(~any(abs(components.directionsAll.eigenvectors) <= minAutovectorMastNoBracket,2) & ~any(components.directionsAll.eigenvalues <= minAutovalueMastNoBracket,2) ...
                & ~any(abs(components.directionsAll.eigenvectors) >= maxAutovectorMastNoBracket,2) & ~any(components.directionsAll.eigenvalues >= maxAutovalueMastNoBracket,2));

% possibleMasts = possibleMasts(~ismember(possibleMasts,components.wall2)); % denoising
% Adding neighbours
neighbours    = vx.neighbours_rows(possibleMasts,:);
neighbours    = reshape(neighbours,numel(neighbours),1);
neighbours    = neighbours(~isnan(neighbours));
possibleMasts = unique([possibleMasts; neighbours]);
% possibleMasts = possibleMasts(~ismember(possibleMasts,components.wall2));

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(possibleMasts,:), 'w', 'MarkerSize',50);
% hold on; pcshow(vx.Location(components.wall2,:), 'y', 'MarkerSize',50);

%% Clustering and checking masts without brackets
idx                         = dbscan(vx.Location(possibleMasts,:),1.5*vx.grid,1);
[num]                       = groupcounts(idx); 
[~, clustersMastsNoBracket] = sort(num,'descend');

% Checking clusters with CheckElement and with the mean of eigenvalues of  its voxels
isElement = false(size(clustersMastsNoBracket));
for i = 1:numel(clustersMastsNoBracket)
    isElement(i) = CheckElement(vx.Location(possibleMasts(idx == clustersMastsNoBracket(i)),:), mastNoBracketModel); % & mean(components.directionsAll.eigenvalues(ismember(components.notTrack, possibleMasts(idx == clustersMastsNoBracket(i))),1),1) > 0.55;
end

% i = 0;
% i = i+1;
% hold on;  pcshow(vx.Location(possibleMasts(idx == clustersMastsNoBracket(i)),:), 'y', 'MarkerSize',50);
% hold on;  pcshow(vx.Location(components.notTrack(ismember(components.notTrack, possibleMasts(idx == clustersMastsNoBracket(i)))),:), 'g', 'MarkerSize',50);

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(possibleMasts(ismember(idx,clustersMastsNoBracket(isElement))),:), 'r', 'MarkerSize',50);

% Only clusters that are masts without bracket
clustersMastsNoBracket = clustersMastsNoBracket(isElement);

%% Adding points
% Adding points to each cluster by distance to add part of the cables
% but only point with higher Z than its min
% clustering and chossing the cluster with more points of the original
% cluster
% Saving each mast without bracket
mastsNoBrackets = cell(size(clustersMastsNoBracket));
for i = 1:numel(clustersMastsNoBracket)
    inRange = find(pdist2(vx.Location(:,1), mean(vx.Location(possibleMasts(idx == clustersMastsNoBracket(i)),1))) < (mastNoBracketModel.dimensions(1) + mastNoBracketModel.toleranceDimensions(1))/2 &...
                   pdist2(vx.Location(:,2), mean(vx.Location(possibleMasts(idx == clustersMastsNoBracket(i)),2))) < (mastNoBracketModel.dimensions(2) + mastNoBracketModel.toleranceDimensions(2))/2 &...
                   vx.Location(:,3) > min(vx.Location(possibleMasts(idx == clustersMastsNoBracket(i)),3)) + 2);
    
    inRange = unique([inRange;possibleMasts(idx == clustersMastsNoBracket(i))]); % points in range and points extracted
    inRange = inRange(ismember(inRange, components.notTrack)); % % Denoising track
    
    % Clustering
    aux     = dbscan(vx.Location(inRange,:), 3*vx.grid,1);
    [num]   = groupcounts(aux); 
    [~, clustersThisMast] = sort(num,'descend');
    
    % Looking for the cluster with more voxels of this mast
    maxGroup = zeros(size(clustersThisMast));
    for j = 1:numel(clustersThisMast)
        maxGroup(j) = sum(ismember(inRange(aux==clustersThisMast(j)), possibleMasts(idx == clustersMastsNoBracket(i))));
    end
    [~,maxGroup] = max(maxGroup);
    
    mastsNoBrackets{i} = inRange(aux == clustersThisMast(maxGroup)); % Saving voxels
end

% hold on; pcshow(vx.Location(inRange,:), 'y', 'MarkerSize',50);
% hold on; pcshow(vx.Location(inRange(aux == clustersThisMast(1)),:), 'w', 'MarkerSize',50);
% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,mastsNoBrackets{:}),:), 'g', 'MarkerSize',50);

% If there are not masts
if isempty(mastsNoBrackets)
    components.masts      = [];
    components.roughMasts = [];
    return;
end

%% Masts with brackets
minAutovectorMast = [NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN];
minAutovalueMast  = [NaN,NaN,NaN];
maxAutovectorMast = [0.5,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN];
maxAutovalueMast  = [NaN,NaN,NaN];

masts = components.notTrack(~any(abs(components.directionsAll.eigenvectors) <= minAutovectorMast,2) & ~any(components.directionsAll.eigenvalues <= minAutovalueMast,2) ...
                & ~any(abs(components.directionsAll.eigenvectors) >= maxAutovectorMast,2) & ~any(components.directionsAll.eigenvalues >= maxAutovalueMast,2));
            
masts    = masts(~ismember(masts, components.wall2)); % Denoising. Antes estaba con wall1
masts    = masts(vx.Location(masts,3) >0);
mastsToo = vx.neighbours_rows(masts,:); % Adding neighbours
mastsToo = reshape(mastsToo,numel(mastsToo),1);
mastsToo = mastsToo(~isnan(mastsToo));
masts    = unique([cat(1,mastsNoBrackets{:}); mastsToo; masts]); % masts, neighbours and mast without brackets
masts    = masts(~ismember(masts,[components.track; components.wall2]));           

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(masts,:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,mastsNoBrackets{:}),:), 'w', 'MarkerSize',50);

%% Clustering masts
idx           = dbscan(vx.Location(masts,:),1.5*vx.grid,1);
[num]         = groupcounts(idx); 
[~, clusters] = sort(num,'descend');

% masts' clusters have points of mastsNoBrackets
withMast = zeros(size(clusters));
for i = 1:numel(clusters)    
    for j = 1:numel(clustersMastsNoBracket)
        if any(ismember(masts(idx == clusters(i)),mastsNoBrackets{j}))
            withMast(i) = j;
        end
    end
end

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(masts,:), 'r', 'MarkerSize',50);
% hold on; pcshow(vx.Location(masts(ismember(idx, clusters(withMast~=0))),:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,mastsNoBrackets{:}),:), 'w', 'MarkerSize',50);

%% Sorting by X
meanX = zeros(size(clustersMastsNoBracket));
for i = 1:numel(clustersMastsNoBracket)
    meanX(i) = mean(vx.Location(mastsNoBrackets{i},1));
end
[~, order] = sort(meanX,'ascend');

%% Saving
components.masts      = [];
components.roughMasts = [];
for i = 1:numel(clustersMastsNoBracket)
    components.roughMasts{i} = mastsNoBrackets{order(i)};
    components.masts{i}      = unique([mastsNoBrackets{order(i)};masts(ismember(idx, clusters(withMast == order(i))))]);
end

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% i = 0;
% i = i+1
% hold on; pcshow(vx.Location(components.masts{i},:), 'r', 'MarkerSize',50);
% hold on; pcshow(vx.Location(components.roughMasts{i},:), 'g', 'MarkerSize',50);

% hold on; pcshow(vx.Location(components.wall,:), 'y', 'MarkerSize',50);
end

