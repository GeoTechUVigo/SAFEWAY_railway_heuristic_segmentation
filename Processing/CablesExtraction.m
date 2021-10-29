function [components] = CablesExtraction(vx, components, cableModel, step, keepingSeeds, axis, minLength, marginSearch)
% Extraction of cables. SE CAMBIO EL NO BORRAR ALREDERODES
% 
% All masts indexes are selected
% 
% Using the result of PcaLocal, all voxels with a X component of its first 
% eigenvector higher than 0.5 in notTrack and with Z > 0 (mean trajectory)
% are selected, removing masts voxels.
% 
% Voxels are clusterd using Linking() function.
% 
% Sort, low voxels density and low linear clusters are deleted.
% 
% Cables are classified in: cathenary, contact or others.
% A cloud only with cables is rasterized from bird's eye view and
% binarized, getting a  image of the cables. 
% Each cable is analyzed, making it wider adding to it its close pixels.
% Then, common pixels between this cable and the others are analized. They
% are pair if they are the ones with the most % of pixels in common,
% measured in the sortest one, only if it is greater than 50%. Next, the
% mean Z of common pixels is evaluated and the highest one is classified as
% catenary and the other as contact wire.
% 
% Cables that share the same pair are merged as they are the same cable.
%
% Other cables are analyzed. They have to be in contact with any mast (no 
% bracket). If not they are deleted.
%
% In droppers and rails function cables might be modified.
% -------------------------------------------------------------------------
% INPUTS:
%
% vx : Voxels. Cloud 
% 
% components : cell. Cell with elements extracted
% 
% cableModel :Element. Cable's model
% 
% step : numeric. Width of each section in Linking function.
%
% keepingSeeds : numeric. Meters that a seed is keep in Linking function
% 
% axis : vector. Dimension to take into account in Linking function
% 
% minLength: numeric. Min length of a cable
% 
% marginSearch : numeric. Margin of search of each cable to make its
%                cableRange
%                
% -------------------------------------------------------------------------
% OUTPUTS:
%
% components.cables.others : cell 1 x num cables other. Indexes of cables
%                            classified as others
%                            
% components.cables.pairs : cell 1 x num of cable pairs. Each cell as 2
%                           cells. {1} has cathenay indexes and {2} contact
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 22/12/2020

%% Deleting masts
if ~isempty(components.masts)
    masts = cat(1,components.masts{:});
else
    masts = [];
end

%% Looking for cables
minAutovector = [0.5,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN];
minAutovalue  = [NaN,NaN,NaN]; 
maxAutovector = [NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN];
maxAutovalue  = [NaN,NaN,NaN];

cables = components.notTrack(~any(abs(components.directionsUp.eigenvectors) <= minAutovector,2) & ~any(components.directionsUp.eigenvalues <= minAutovalue,2) ...
                & ~any(abs(components.directionsUp.eigenvectors) >= maxAutovector,2) & ~any(components.directionsUp.eigenvalues >= maxAutovalue,2));
            
cables = cables(~ismember(cables, masts)); % Denoising. Deleting masts
cables = cables(vx.Location(cables,3) > 0); % Voxels up to trajectory

% figure; pcshow(vx.Location,'b','MarkerSize',50);
% hold on; pcshow(vx.Location(cables,:),'r','MarkerSize',50);

%% Clustering cables by analizing the axis 2 and 3
idx = Linking(vx,cables,step, cableModel.dimensions(2), keepingSeeds,axis);
[num] = groupcounts(idx); 
[~, clusters] = sort(num,'descend');

%% Deleting noise analysing length, linearity and density
cablesNotTrackIdx = find(ismember(components.notTrack, cables)); % To study the linearity

lengthCable = zeros(size(clusters));
eigenvalue = zeros(size(clusters));
density = zeros(size(clusters));
for i = 1:numel(lengthCable)
   lengthCable(i) = sqrt(range(vx.Location(cables(idx == clusters(i)),1))^2 + range(vx.Location(cables(idx == clusters(i)),2))^2);
   eigenvalue(i)   = mean(components.directionsUp.eigenvalues(cablesNotTrackIdx(idx == clusters(i)),1));
   density(i)     = sum(idx == clusters(i)) / lengthCable(i);
end

clusters = clusters(lengthCable > minLength & eigenvalue > 0.70 & density > 0.7*(1/vx.grid));

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cables,:), 'r', 'MarkerSize', 50);
% hold on; pcshow(vx.Location(cables(ismember(idx, clusters(lengthCable > minLength & autovalue > 0.70 & density > 5))),:), 'g', 'MarkerSize', 50);

% i = 0;
% i = i+1
% color = [rand(1),rand(1),rand(1)*0.5];
% hold on; pcshow(vx.Location(cables(idx == clusters(i)),:),color,'MarkerSize',50);


% [sorted, order] = sort(density,'descend');
% i = 0;
% i = i+1
% hold on; pcshow(vx.Location(cables(idx == clusters(order(i))),:),'y','MarkerSize',200)

%% Classifying
% In the first column 1 --> catenary, 2--> contact, 3--> other
% In the second their cluster pair

% Doing raster of cables
idxCables = idx(ismember(idx,clusters)); % idx only considering real cables voxels
if isempty(idxCables)
    components.cables        = [];
    components.cables.pairs  = [];
    components.cables.others = [];
    return;
end

vxCables = select(vx, cables(ismember(idx,clusters))); % cloud only with cables
raster   = raster2D(vxCables,marginSearch*2/3); % raster with bigger grid

classification = zeros(numel(clusters),2);

% Raster image of each cable
cable = cell(1,numel(clusters));
for i = 1:numel(clusters)
    
    % Pixels of this cable
    cable{i} = false(size(raster.intensity_image));     
    idxCable = find(idxCables == clusters(i));
    for j = 1:numel(raster.parent_idx) % All pixiles with voxels
        if any(ismember(raster.parent_idx{j}, idxCable)) % any voxels of this pixel is a cable
            cable{i}(raster.indices(j)) = true;
        end
    end
end

% Going through all clusters, calculating its cableRange (pixels of this
% cable and pixels close to it) and comparing it with the other cables
for i = 1:numel(clusters)
    % Desviation in pixels close to this cable to do the comparation of
    % shared pixels between cables
    maxRange = 1; % only 1 pixel of deviation because the grid is marginSearch*2/3.

    pixelsRow = size(raster.intensity_image,1);

    % Pixel of this cable and pixel close to it in range (looking for at both sides of this cable)
    cableRange = false(size(raster.intensity_image)); 
    for j = 1:size(raster.intensity_image,2) % All columns in raster

        if any(cable{i}(:,j)) % any pixels of this colum is a cable

            % Desviation at right
            cablesNotTrackIdx = find(cable{i}(:,j),1,'first') - maxRange; % first pixel in column (more at right)

            % keeping ranges in this column
            if cablesNotTrackIdx < 1
                cablesNotTrackIdx = 1;
            end

            % Same in left
            c = find(cable{i}(:,j),1,'last') + maxRange;

            if c > pixelsRow
                c = pixelsRow;
            end

            cableRange(pixelsRow*(j-1) + cablesNotTrackIdx :  pixelsRow*(j-1) + c) = true;
        end
    end
    
    % Comparing this cableRange with the other cables
    inCommon = zeros(numel(clusters),1);
    for j = 1:numel(clusters)
        if j ~=i % if it is not the same cable
            % Comparing --- > common pixels / by the sorter one. 
            % sum(sum()) because it is a matrix
            inCommon(j,1) = sum(sum(cableRange & cable{j})) / min(sum(sum(cable{i})), sum(sum(cable{j})));
        end
    end
        
    if any(inCommon > 0.5) % If any cable shares more than the 50% of its pixels with the cable i

        [~,j] = max(inCommon); % cluster(j) is its pair
        
        % Calculating which is taller to classify but only common parts
        
        % Only considering the part of the cables that they have in the same
        % (x,y)
        bothCables        = false(size(raster.intensity_image)); 
        aux               = any(cableRange & cable{j},1); % All row with both cables
        bothCables(:,aux) = true; % image with true in rows in both cables
        
        voxelsCables = cat(1,raster.parent_idx{ismember(raster.indices,find(bothCables))}); % all voxels in rows with common pixels
        cableI       = voxelsCables(ismember(voxelsCables, find(idxCables == clusters(i)))); % part of cable i with same (x,y) than j
        cableJ       = voxelsCables(ismember(voxelsCables, find(idxCables == clusters(j)))); % part of cable j with same (x,y) than i

        if  mean(vxCables.Location(cableI,3)) > mean(vxCables.Location(cableJ,3)) % cathernaty
            classification(i,1) = 1;
            classification(i,2) = j;
        else  % contact
            classification(i,1) = 2;
            classification(i,2) = j;
        end
    else % other
            classification(i) = 3;
    end
end

% hold on; pcshow(vx.Location(cables(ismember(idx, clusters(classification(:,1) == 1))),:),'r','MarkerSize',50)
% hold on; pcshow(vx.Location(cables(ismember(idx, clusters(classification(:,1) == 2))),:),'g','MarkerSize',50)
% hold on; pcshow(vx.Location(cables(ismember(idx, clusters(classification(:,1) == 3))),:),'y','MarkerSize',50)
% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% i = 0;
% i = i+1;
% hold on; pcshow(vx.Location(cables(ismember(idx, clusters(i))),:),'y','MarkerSize',50)
% hold on; pcshow(vx.Location(cables(ismember(idx, clusters(classification(i,2)))),:),'r','MarkerSize',50)
% hold on; pcshow(vx.Location(cables(ismember(idx, clusters(7))),:),'g','MarkerSize',50)

%% Mergin cutted cables that are from the same pair
for i = 1:length(clusters)
    if classification(i,2) ~= 0 && sum(ismember(classification(:,2), classification(i,2))) > 1 % More than one cluster with this pair
        aux = ismember(classification(:,2), classification(i,2)); % Positions in clusters that have the same pair classification(i,2)
        idx(ismember(idx,clusters(aux))) = clusters(i); % Changing the idx of clusters with this pair
        aux(i) = false; % not taking in acount this cluster
        classification(classification(i,2),2)   = i; % The pair will have as a pair the cluster that is not deleted
        classification(aux,:) = 0; % Deleting from classification the other clusters
    end  
end

%% Other cables are in contact with masts
others = find(classification(:,1) == 3); % Position in classification of other cables
if ~isempty(masts)
    for i = others'
        if min(min(pdist2(vx.Location(cables(idx == clusters(i)),:), vx.Location(masts,:)),[],1)) > 1 % min distance of this cable to any mast
            classification(i,:) = 0; % Deleting  cable
        end    
    end
else
    classification(others,:) = 0; % Deleting  cable
end

%% Saving
components.cables = [];
components.cables.pairs = [];
components.cables.others = [];
for i = 1:numel(clusters)
    if classification(i,1) == 1 % it is a catenary
        components.cables.pairs{numel(components.cables.pairs) +1}{1} =  cables(idx == clusters(i)); % catenary
        components.cables.pairs{numel(components.cables.pairs)}{2}    =  cables(idx == clusters(classification(i,2))); % its contact pair
    elseif classification(i,1) == 3 % other 
        components.cables.others{numel(components.cables.others) +1} = cables(idx == clusters(i));
    end
end

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% i = 0;
% i = i+1;
% hold on; pcshow(vx.Location(components.cables.pairs{i}{1},:),'g','MarkerSize',50);
% hold on; pcshow(vx.Location(components.cables.pairs{i}{2},:),'y','MarkerSize',50);
% 
% i = 0;
% i = i+1
% hold on; pcshow(vx.Location(components.cables.others{i},:),'r','MarkerSize',50);
end