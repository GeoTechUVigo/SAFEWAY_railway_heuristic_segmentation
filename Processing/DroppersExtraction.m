function [components] = DroppersExtraction(vx, components, dropperModel, distDbscan)
% Droppers extraction
% 
% Possible droppers. Selecting voxels in notTrack that are neither masts
% nor cables. Only voxels between cables in Y axis taking into account a
% margin.
%
% Adding its neighbours to add cable voxels to make groups, but saving it
% in a different variable.
%
% Voxels are clustered and sorted by X.
%
% If a cluster has voxels of a pair of cables (both), because neighbours of 
% possible droppers were added, this cluster is checked with dropperModel.
% If it is a dropper, it is saved as a dropper of that pair of cables, but
% only its voxel in possibleDroppers (without any voxel of cables).
%
% Pair of cables without any dropper are deleted from pair of cables and
% saved in other cables
%
%--------------------------------------------------------------------------
% INPUTS:
%
% vx : Voxels. Cloud 
% 
% components : cell. Cell with elements extracted
% 
% dropperModel : Element. Dropper's model
% 
% distDbscan : numeric. Distance used in dbscan() clustering.
%                
% -------------------------------------------------------------------------
% OUTPUTS:
% 
% components.droppers : cell 1 x num of cable pairs. Each cell has a cell
%                       with cells. Each one has the index of each dropper
%                       of a pair of cables.
%
% components.cables.others : cell 1 x num cables other. Updated.
%                            
% components.cables.pairs : cell 1 x num of cable pairs. Updated.
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 22/12/2020

%% Possible droppers
% Working with notTrack cloud and deleting masts and cables, and droppers are between cables
if ~isempty(components.cables.pairs)
    cables = cat(1,components.cables.pairs{:});
    cables = cat(1,cables{:});
else % if there are not pair of cables, there are not droppers
    components.droppers = [];
    return;
end
if ~isempty(components.masts)
    masts = cat(1,components.masts{:});
else
    masts = [];
end
if ~isempty(components.cables.others)
    cablesOthers = cat(1,components.cables.others{:});
else
    cablesOthers = [];
end
possibleDroppers = components.notTrack(~ismember(components.notTrack,masts) & ~ismember(components.notTrack,cables) & ~ismember(components.notTrack,cablesOthers));

cables = cat(2,components.cables.pairs{:});
catenary = cell(1,numel(components.cables.pairs));
contact = cell(1,numel(components.cables.pairs));
for i = 1:numel(components.cables.pairs) % Saving caterany and contact to last check. Each Dropper has to be in contact with both.
    catenary{i} = components.cables.pairs{i}{1};
    contact{i}  = components.cables.pairs{i}{2};
end
cables = cat(1,cables{:});
possibleDroppers = possibleDroppers((vx.Location(possibleDroppers,3) > min(vx.Location(cables,3)) & vx.Location(possibleDroppers,3) < max(vx.Location(cables,3))) ...
                   & (vx.Location(possibleDroppers,2) >= min(vx.Location(cables,2)) - 0.5 & vx.Location(possibleDroppers,2) <= max(vx.Location(cables,2)) + 0.5));

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(possibleDroppers,:), 'r', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,contact{:}),:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,catenary{:}),:), 'y', 'MarkerSize',50);

%% Adding neighbours
droppersToo = vx.neighbours_rows(possibleDroppers,:);
droppersToo = reshape(droppersToo,numel(droppersToo),1);
droppersToo = droppersToo(~isnan(droppersToo));
droppers    = unique([possibleDroppers; droppersToo]);

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(droppers,:), 'r', 'MarkerSize',50);
% hold on; pcshow(vx.Location(possibleDroppers,:), 'w', 'MarkerSize',50);

%% Clustering
if ~isempty(droppers)
    idx           = dbscan(vx.Location(droppers,:),distDbscan,1);
    [num]         = groupcounts(idx); 
    [~, clusters] = sort(num,'descend');

    %% Sorting by X
    meanX = zeros(size(clusters));
    for i = 1:numel(clusters)
        meanX(i) = mean(vx.Location(droppers(idx == clusters(i)),1));
    end
    [~, order] = sort(meanX,'ascend');
    clusters   = clusters(order);

    %% Saving only cluster in contact with a pair of cables. Only saving the voxels that are not part of a cable
    components.droppers = cell(1, numel(components.cables.pairs));
    for i = 1:numel(clusters)
        for j = 1: numel(components.cables.pairs)
            if any(ismember(droppers(idx == clusters(i)), catenary{j})) && any(ismember(droppers(idx == clusters(i)), contact{j}))
                aux = droppers(idx == clusters(i));
                aux = aux(ismember(aux,possibleDroppers)); % dropper without any voxel of cables
                if isempty(aux) % if there are not any point, it means that all this points are cable, so it is not dropper
                    break;
                end
                if CheckElement(vx.Location(aux,:), dropperModel) % Checking element
                    components.droppers{j}{numel(components.droppers{j}) + 1} =  aux;
                end
                break; % next cluster, because this cluster has been assigned as a dropper of this pair of cables
            end
        end
    end
  
%     hold on; pcshow(vx.Location(droppers,:), 'w', 'MarkerSize',50);
%     hold on; pcshow(vx.Location(droppers(idx == clusters(i)),:), 'g', 'MarkerSize',50);
%     hold on; pcshow(vx.Location(contact{1},:), 'r', 'MarkerSize',50);
%     hold on; pcshow(vx.Location(catenary{1},:), 'r', 'MarkerSize',50);
%     hold on; pcshow(vx.Location(contact{2},:), 'r', 'MarkerSize',50);
%     hold on; pcshow(vx.Location(catenary{2},:), 'r', 'MarkerSize',50);


% figure; pcshow(vx.Location, 'b', 'MarkerSize', 50)
% hold on; pcshow(vx.Location(droppers,:), 'w', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,components.droppers{1}{:}),:), 'r', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,components.droppers{2}{:}),:), 'y', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,components.droppers{3}{:}),:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,components.droppers{4}{:}),:), 'w', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,components.cables.pairs{1}{1}),:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,components.cables.pairs{1}{2}),:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,components.cables.pairs{2}{1}),:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1,components.cables.pairs{2}{2}),:), 'g', 'MarkerSize',50);
% 
% i = i+1
% hold on; pcshow(vx.Location(droppers(idx == clusters(i)),:), 'g', 'MarkerSize',50);

else
    components.droppers = cell(1,numel(components.cables.pairs));
end

%% If there are not droppers it means that there are not catenary and contact, so that cables are changed to others cables
keep = ~cellfun(@ isempty,components.droppers);

components.droppers = components.droppers(keep); % deleting empty structures

% Saving that cables in other cables
if ~isempty(masts)
    for i = find(keep == false)

        % Other cables are in contact with masts
        if min(min(pdist2(vx.Location(components.cables.pairs{i}{1},:), vx.Location(masts,:)),[],1)) < 1 % min distance of this cable to any mast
            components.cables.others{numel(components.cables.others)+1} = components.cables.pairs{i}{1};
        end
        if min(min(pdist2(vx.Location(components.cables.pairs{i}{2},:), vx.Location(masts,:)),[],1)) < 1
            components.cables.others{numel(components.cables.others)+1} = components.cables.pairs{i}{2};
        end
    end
end

components.cables.pairs = components.cables.pairs(keep); % deleting cables that are not pairs
end