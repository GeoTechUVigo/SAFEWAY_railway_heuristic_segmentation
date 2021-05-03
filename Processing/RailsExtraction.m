function [components] = RailsExtraction(vx, components, railPairModel, marginSearch, heightFromGround)
% Rails extraction using the catenary or contact cable.
% 
% First, vx is rasterized from bird´s eye view. Columns of the raster
% image correspond to the Y axis in the cloud, and rows to X.
%
% In each pair of cables, the longest is selected. In the selected cable
% is studied the distance to all masts and catenaries. The objective is to
% know if the start or the end of a cable is in contact with a mast, which
% brackets are in contact with that cable and which mastil without bracket
% is in contact with that cable. Besides, the number of droppers between
% its first/last bracket in contact and the start/end of the cable is also
% evaluated. The objective is to know which part of the cable has rails 
% under it.
% If the cable is in contact with any mast and it is not in its begging nor
% end there are not rails.
% If there are not enough droppers before/after its last/first bracket in
% contact, of if before/after that brackets the cable is in contact with a
% mast, it means that the cable end/start there, so that part before/after
% the brackets does not has rails.
%
% pixels of cable. A binary image using the raster of the cloud with true in
% voxels where there are any part of the cable considered is created. Might
% be that the cable is not continuous because any errors, so in columns
% without any pixel of the cable between the range of the cable are matched
% to its previos column.
%
% Voxels with rough rails. Colum by colum in the raster image (X axis in
% the cloud), pixels at right and at left are selected. Voxels of the track
% in this pixels are voxels with the right and the left rail of this cable.
%
% Filtering rails. Each rail is cleaned, trying to delete voxels that are 
% not rail voxels. For that, the rail is oriented and sectioned. In each 
% section, an histogram in Z is done looking for the Z of the ground. The
% Z of the ground is the level of Z with more points. Voxels of the ground
% are removed.
%
% Pair of rails. To delete parts of the ground that might be classified as
% rail because of its Z level, a comparation between both rail is done.
% That comparation is made in a raster image of both rails. In each colum,
% one rail has to have in a certain distance pixels of pair rail. Only
% voxels that fullfit this are classified as rails.
%
% Pair of cables without rails are saved as other cables and its droppers
% deleted.
%
% Other cables must be in contact with any mast.
%--------------------------------------------------------------------------
% INPUTS:
%
% vx : Voxels. Cloud 
% 
% components : cell. Cell with elements extracted
% 
% railPairModel : Element. Both rails' model as a pair
% 
% marginSearch : numeric. Margin of search from the point where is supossed
%                to be a rail based on the catenary position.
% 
% heightFromGround : numeric. Distance between the ground and the start of
%                    the rail.
%
% -------------------------------------------------------------------------
% OUTPUTS:
% 
% components.rails : cell 1 x num of cable pairs. Each cell has a cell
%                    with  2 cells. Each one has the index of each rail.
%                    1 --> left rail. 2 --> right rail.
%
% components.cables.others : cell 1 x num cables other. Updated.
%                            
% components.cables.pairs : cell 1 x num of cable pairs. Updated.
%
%  components.droppers : cell 1 x num of cable pairs. Updated.
%
% components.masts  : cell 1 x num of masts. Updated.
% 
% components.roughMasts  : cell 1 x num of masts. Updated.
%
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 23/12/2020

%% Looking for rails for each pair of cables (catenary and contact)
components.rails = [];

if isempty(components.cables.pairs) % if there are not pairs of cables, there are not rails
    components.rails = [];
    return;
end
    
raster = raster2D(vx, vx.grid); % raster of vx

for i = 1:numel(components.cables.pairs)
    %% Choosing voxels of the cable with rails
    % The begging and the end part of cables do not have rails.
    % Those parts are between a mast (components.roughMast) and the
    % closest bracket.

    % Neighbours of this cable
    longest = zeros(2,1);
    for j = 1:2
        longest(j) = range(vx.Location(components.cables.pairs{i}{j},1));
    end
    [~,longest] = max(longest);

    neighbours = vx.neighbours_rows(components.cables.pairs{i}{longest},:);
    neighbours = reshape(neighbours, [],1);
    neighbours = neighbours(~isnan(neighbours));

    % Start and end point of this cable
    [~,startCable] = min(vx.Location(components.cables.pairs{i}{longest},1));
    startCable     = startCable(1);
    startCable     = vx.Location(components.cables.pairs{i}{longest}(startCable),:);

    [~,endCable] = max(vx.Location(components.cables.pairs{i}{longest},1));
    endCable     = endCable(1);
    endCable     = vx.Location(components.cables.pairs{i}{longest}(endCable),:);

    % Saving which masts and brackets are in contact with this cable 
    mastContactStart = false(numel(components.masts),1);
    mastContactEnd   = false(numel(components.masts),1);
    bracketContact   = false(numel(components.masts),1);
    mastContactAny   = false(numel(components.masts),1);

    for j = 1:numel(components.masts)

        % Distance because sometimes there are not contact because of a bad extraction
        % Distance from this cable to this mast
        [~, dist] = knnsearch(vx.Location(components.cables.pairs{i}{longest},:), vx.Location(components.roughMasts{j},:));
        if min(dist) < 1 % If any part of this cable is in contact with this mast
            mastContactAny(j) = true;
        end

        if min(pdist2(startCable, vx.Location(components.roughMasts{j},:))) < 1 % If the start of the cable is in contact
            mastContactStart(j) = true;
        elseif min(pdist2(endCable, vx.Location(components.roughMasts{j},:))) < 1 % If the end of the cable is in contact
            mastContactEnd(j) = true;
        elseif any(ismember(neighbours, components.masts{j}(~ismember( components.masts{j},components.roughMasts{j})))) % Analyzing contact with brakets (no masts)
            bracketContact(j) = true;
        end
    end

    % The num of droppers since the beggining of the cable to the first
    % bracket contact is analysed in order to know if the cable
    % start/ends just before/after this bracket. If there are not
    % enough droppers it is the start/end of the cable.
    droppersFirst = 0;
    droppersLast  = 0;

    bracket = find(bracketContact,1,'first'); % index of the first braket in contact with this cable 
    if ~isempty(bracket)

        % num of droppers in this cable before its first bracket
        bracket = mean(vx.Location(components.masts{bracket}),1);
        for j = 1:numel(components.droppers{i})
            if any(vx.Location(components.droppers{i}{j},1) < bracket)
                droppersFirst = droppersFirst +1;
            else
                break; % they are sorted by X 
            end
        end

        % num of droppers in this cable after its last bracket
        bracket = find(bracketContact,1,'last');
        bracket = mean(vx.Location(components.masts{bracket}),1);
        for j = numel(components.droppers{i}):-1:1
            if any(vx.Location(components.droppers{i}{j},1) > bracket)
                droppersLast = droppersLast +1;
            else
                break;
            end
        end

        % If there are less than 2 droppers the cable will be in contact
        % with a mast in the next/previous section, so it starts or ends.
        droppersFirst = droppersFirst < 2;
        droppersLast  = droppersLast < 2;

    else % if there are not brackets in contact with this cable it cannot be analysed by num of droppers
        droppersFirst = false;
        droppersLast  = false;
    end

    % Choosing the voxels of the cable
    if ~all(mastContactAny == mastContactStart | mastContactAny == mastContactEnd) % it is neither catenary nor contact because it is in contact with any mast through its middle
        components.rails{i} = [];
        continue;
        
    elseif (any(mastContactStart) || droppersFirst) && (any(mastContactEnd) || droppersLast) % the cable starts and ends in this section
        if sum(bracketContact) > 1 % If the cable start and end in this section must be in contact with at least 2 brackets
            cableVoxels = components.cables.pairs{i}{longest}(vx.Location(components.cables.pairs{i}{longest},1) < mean(vx.Location(components.masts{find(bracketContact,1,'last')},1)) & vx.Location(components.cables.pairs{i}{longest},1) > mean(vx.Location(components.masts{find(bracketContact,1,'first')},1)));
        else 
            components.rails{i}= [];
            continue;
        end
    elseif any(mastContactStart) || droppersFirst % The cable starts in this section
        if any(bracketContact)
            cableVoxels = components.cables.pairs{i}{longest}(vx.Location(components.cables.pairs{i}{longest},1) > mean(vx.Location(components.masts{find(bracketContact,1,'first')},1)));
        else % The cable starts but it does not reach the center of the railway, so there are not rails
            components.rails{i}= [];
            continue;
        end
    elseif any(mastContactEnd) || droppersLast % The cable ends in this section
        if any(bracketContact)
            cableVoxels = components.cables.pairs{i}{longest}(vx.Location(components.cables.pairs{i}{longest},1) < mean(vx.Location(components.masts{find(bracketContact,1,'last')},1)));
        else % The cable end but it does not come from the center of the railway, so there are not rails
            components.rails{i} = [];
            continue;
        end
        
    else % All the cable

        cableVoxels = components.cables.pairs{i}{longest};
    end

    %% pixel of cable
    cable = false(size(raster.intensity_image));     
    for j = 1:numel(raster.parent_idx) % All pixiles with voxels
        if any(ismember(raster.parent_idx{j}, cableVoxels)) % any voxels of this pixel is a cable
            cable(raster.indices(j)) = true;
        end
    end

    % The cable may not be continuos because of mistakes in its indexes.
    % Columns whitout any pixel are modified and equalized to the previous
    % column

    lastCable = find(any(cable,1),1,'last');
    for j = 2:lastCable
        if ~any(cable(:,j))
            cable(:,j) = cable(:,j-1);
        end
    end

    %% Voxels with rough rails

    % Desviation in pixels between the cable and the rails
    maxRange = ceil(((railPairModel.dimensions(2)+railPairModel.toleranceDimensions(2))/2 + marginSearch/2)/vx.grid);
    minRange = fix(((railPairModel.dimensions(2)-railPairModel.toleranceDimensions(2))/2 - marginSearch/2)/vx.grid);

    pixelsRow = size(raster.intensity_image,1);

    % Initializing. It will have the position in parent_idx where there
    % are possible voxels of rails
    rail{1} = false(size(raster.parent_idx)); % left
    rail{2} = false(size(raster.parent_idx)); % right

    for j = 1:size(raster.intensity_image,2) % All columns in raster

        if any(cable(:,j)) % any pixels of this colum is a cable

            % Ranges of pixels of right rail 
            a = find(cable(:,j),1,'first'); % first pixel in column (more at right)
            b = a - maxRange; % max desviation
            a = a - minRange; % min desviation

            % keeping ranges in this column
            if a < 1
                a = 1;
                b = 1;
            elseif b < 1
                b = 1;
            end

            % Same in left rail
            c = find(cable(:,j),1,'last');
            d = c + maxRange;
            c = c + minRange;

            if c > pixelsRow
                c = pixelsRow;
                d = pixelsRow;
            elseif d > pixelsRow
                d = pixelsRow;
            end

            % raster.indices has the idx of the pixels that are not
            % empty, sorted in the same order as raster.parent_idx, so I
            % save the rows where there are the idx of pixels selected by 
            %sum and rest the desviation from the pixel of
            % the cable (j)
            rail{2}(ismember(raster.indices, pixelsRow*(j-1) + b :  pixelsRow*(j-1) + a)) = true; % right
            rail{1} (ismember(raster.indices, pixelsRow*(j-1) + c :  pixelsRow*(j-1) + d)) = true; % left
        end
    end

    if isempty(rail{2}) || isempty(rail{2})
        components.rails{i} = [];
        continue;
    end
    % Idx of voxels
    rail{1} = cat(1,raster.parent_idx{rail{1}}); % idx of voxels
    rail{1} = rail{1}(ismember(rail{1}, components.track)); % in track
    rail{2} = cat(1,raster.parent_idx{rail{2}}); % idx of voxels
    rail{2} = rail{2}(ismember(rail{2}, components.track)); % in track  

%     figure; pcshow(vx.Location);      
%     hold on; pcshow(vx.Location(rail{1},:),'r');   
%     hold on; pcshow(vx.Location(rail{2},:),'y');
%     hold on; pcshow(vx.Location(components.cables.pairs{i}{longest},:),'y', 'MarkerSize',100);
%     hold on; pcshow(vx.Location(cableVoxels,:),'g', 'MarkerSize',100);

    %% Filtering rails

    for j = 1:2

        railLocation = vx.Location(rail{j},:) - mean( vx.Location(rail{j},:));
        railLocation = railLocation * PcaFlattering(railLocation); 

        minX = min(railLocation(:,1));
        maxX = max(railLocation(:,1));

        railElement = false(size(railLocation,1),1);

        step = 0.5;
        for k = minX:step:maxX

            minSec = k;
            maxSec =  minSec + step;

            railSec = find(railLocation(:,1) >= minSec & railLocation(:,1) < maxSec);

%             figure;histogram(railLocation(railSec,3),'BinWidth',0.02);
            [N,edges] = histcounts(railLocation(railSec,3),'BinWidth',0.02);
%             filter = edges(islocalmax(N));
%             if isempty(filter)
%                 filter = 0;
%             else
%                 filter = filter(1);
%             end                

            [~,filter] = max(N);
            filter = edges(filter); % ground
            if filter > 0 % 0 is the mean of all the rail. If filter > 0 it means that it is not the ground level
                filter = 0;
            end

            railElement(railSec(railLocation(railSec,3) > filter + heightFromGround & railLocation(railSec,3) < filter + railPairModel.dimensions(3)))= true;
        end
 
%         figure; pcshow(railLocation);
%         hold on; pcshow(railLocation(railElement,:), 'r', 'MarkerSize', 50);

        rail{j} = rail{j}(railElement);
    end

    %% Pair of rails
%     figure; pcshow(vx.Location, 'b');
%     hold on; pcshow(vx.Location(rail{1},:),'r');
%     hold on; pcshow(vx.Location(rail{2},:),'y');

    % Pixels of each rail
    rightPixels = false(size(raster.intensity_image));
    leftPixels  = false(size(raster.intensity_image));
    for j = 1:numel(raster.parent_idx) % All pixiles with voxels
        if any(ismember(raster.parent_idx{j}, rail{2})) % any voxels of this pixel (raster.indices(j)) is right rail
            rightPixels(raster.indices(j)) = true;
        elseif any(ismember(raster.parent_idx{j}, rail{1})) % any voxels of this pixel left rail
            leftPixels(raster.indices(j)) = true;
        end  
    end

    % Initializing. It will have the position in parent_idx where there
    % are rails
    rail{1} = false(size(raster.parent_idx));
    rail{2} = false(size(raster.parent_idx)); 

    % Deviation in pixels between rails
    maxRange = ceil(((railPairModel.dimensions(2)+railPairModel.toleranceDimensions(2)))/vx.grid);
    minRange = fix(((railPairModel.dimensions(2)-railPairModel.toleranceDimensions(2)))/vx.grid);

    for j = 1:size(raster.intensity_image,2) % All columns in raster
        if any(leftPixels(:,j)) % any pixels of this column is left rail
            for k = find(leftPixels(:,j))' % Going over all pixels of left rail in this column
                a = k - minRange;
                b = k - maxRange;

                % keeping ranges in this column
                if a < 1
                    a = 1;
                    b = 1;
                elseif b < 1
                    b = 1;
                end

                if any(rightPixels(pixelsRow*(j-1) + b : pixelsRow*(j-1) + a)) % & (abs(raster.height2_image(pixelsRow*(j-1) + b : pixelsRow*(j-1) + a) - raster.height2_image(k,j)) < heightDiff)) % any pixel of right rail in range
                    rail{1}(ismember(raster.indices,pixelsRow*(j-1) + k)) = true;
                    pixels = (pixelsRow*(j-1) + b : pixelsRow*(j-1) + a);
%                     pixels = pixels(abs(raster.height2_image(pixelsRow*(j-1) + b : pixelsRow*(j-1) + a) - raster.height2_image(k,j)) < heightDiff); % Also considering Z level on range
                    rail{2}(ismember(raster.indices, pixels(rightPixels(pixels)))) = true;
                end  
            end
        end
    end

    %% Saving
    if any(rail{1}) && any(rail{2})
        rail{1} = cat(1,raster.parent_idx{rail{1}});
        rail{1} = rail{1}(ismember(rail{1}, components.track)); % idx of voxels

        rail{2} = cat(1,raster.parent_idx{rail{2}});
        rail{2} = rail{2}(ismember(rail{2}, components.track)); % idx of voxels

        components.rails{i}{1} = rail{1};
        components.rails{i}{2} = rail{2};

    else %no rails
        components.rails{i} = [];
    end

%     figure; pcshow(vx.Location, 'b');      
%     hold on; pcshow(vx.Location(rail{1},:),'r');   
%     hold on; pcshow(vx.Location(rail{2},:),'y'); 

end

%% If there are not rails it means that there are not catenary and
% contact, so that cables are changed to others cables, and droppers
% are not droppers
keep = ~cellfun(@ isempty,components.rails);

components.rails    = components.rails(keep); % deleting empty structures
components.droppers = components.droppers(keep); % deleting droppers that are not droppers

% Masts idx to check if they are cables
if ~isempty(components.masts)
    masts = cat(1,components.masts{:});
else
    masts = [];
end

% Saving those cables in other cables
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

%% Once cables are reclassified, deleting masts that are not in contact with any cables.others

if ~isempty(components.cables.others)
    keep = false(numel(components.masts),1);
    cableOthers = cat(1,components.cables.others{:});
    for i = 1:numel(components.masts)
        if min(min(pdist2(vx.Location(components.masts{i},:), vx.Location(cableOthers,:)),[],1)) < 1 % min distance between this mast and any cable other
            keep(i) = true;
        end    
    end
     
else
    keep = false(numel(components.masts),1);
end

components.masts      = components.masts(keep); % deleting
components.roughMasts = components.roughMasts(keep); % deleting 

end