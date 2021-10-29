function [components, status] = Segmentation(vx, traj, sections, idxOriginal, model, status)
% Function to extract indexes of components referred to the original cloud
% using idxOrginial indexes, but making the extracion in vx.
%
% Components are extracted section by section.
%
% Each section is oriented using the trajectory of its section.
%
% Track, wall and roof is extracted.
%
% Points that are neither in in track nor in roof are analyzed using
% LocalPca function. This result is used to extract masts, cables, droppers
% and signal.
%
% Rails are extracted using the catenary or contact cable.
%
% The indexes are recalculated in vx, not in vxSec.
%
% A function is applied to merge all the section. Continuous elements, like
% cables, are evaluated to know if there are contination between sections,
% saving it as the same element. To merge rails and droppers dependent to
% de same pair of cables, the comparation is done with the cables.
%
% Points are recalculated in vx.parent_cloud.
%
% Point are recalculated in the original cloud using idxOriginal indexes.
% 
% -------------------------------------------------------------------------
% INPUTS:
%
% vx : Voxels. Voxelized cloud.
%   
% traj : trajectory. Trajectory of vx
%
% sections : cell array with the indexes of the sections of vx and traj.
%
% idxOriginal : Nx1 numeric. Indexes of vx.parent_cloud in its original
%               .las cloud
%
% model : cell array of Element. Models of element that are going to be
%         extracted.
%
% status : cell array. Varable to save times.
%
% -------------------------------------------------------------------------
% OUTPUTS:
%
% components : cell array of indexes. Indexes of each element segmented
% 
% status : cell array. Execution times.
%
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 28/12/2020

%% Do you want to plot?
wantPlotSec   = false;
wantPlotCloud = false;

%% Times

status.grid          = vx.grid;
status.selectSection = 0;
status.track         = 0;
status.rails         = 0;
status.directions    = 0;
status.masts         = 0;
status.droppers      = 0;
status.signs         = 0;
status.indexVx       = 0;
status.cloudIds      = 0;

status.length = sqrt(range(traj.points(:,1))^2 + range(traj.points(:,2))^2);

componentsVxSec = [];
components   = [];

%% Applying algorithms section by section
for i = 1:numel(sections)
    %% Reseting times that might not be updated
    tSection       = tic;

    tRails         = 0;
    tDirections    = 0;
    tMasts         = 0;
    tDroppers      = 0;
    tSigns         = 0;

    %% Selecting the section and orienting it using the pca trajectory
    tSelectSection = tic;

    % Trajectory
    trajSec  = select(traj, sections{i}.traj);
    meanTraj       = mean(trajSec.points);
    trajSec.points = trajSec.points - meanTraj;
    pcaTraj        = PcaFlattering(trajSec.points);
    trajSec.points = trajSec.points * pcaTraj;

    % Checking if trajPoinjt is in the the correct order
    if trajSec.points(end,1) < trajSec.points(1,1)
        pcaTraj = RotateAxes(pcaTraj, 180, pcaTraj(:,3)); % Rotatin the axes
        trajSec  = select(traj, sections{i}.traj); % Original coordinates
        trajSec.points = trajSec.points - meanTraj; % Coordinates centred
        trajSec.points = trajSec.points * pcaTraj; % Coordinates in the new axes
    end

    % Cloud
    numelIdx = cellfun(@numel, vx.parent_idx);
    idxCloud = cat(1, vx.parent_idx{:});
    idxCloud = ismember(idxCloud, sections{i}.cloud);
    idxSection = zeros(size(numelIdx));
    cont = 1;
    for j = 1:numel(numelIdx)
        nextCont = cont + numelIdx(j);
        if any(idxCloud(cont:nextCont-1))
            idxSection(j) = j;
        end
        cont = nextCont;
    end
    idxSection = idxSection(idxSection ~= 0);

    vxSec = select(vx, idxSection);

    vxSec.Location = vxSec.Location - meanTraj;
    vxSec.Location = vxSec.Location * pcaTraj;

%         figure; pcshow(vx.Location,'b')
%         hold on; pcshow(vx.Location(idxSection,:),'r')

    tSelectSection = toc(tSelectSection);

    %% Segmentation: track and not track
    tTrack           = tic;
    gridExtrackTrack = 0.3;
    subSectionsWidth = 3*vx.grid; % 5
    distWall         = 1;

    componentsVxSec{i} = TrackSegmentation(vxSec, gridExtrackTrack, subSectionsWidth, distWall, trajSec);
    tTrack             = toc(tTrack);

    %% Local Pca of each voxel
    tDirections = tic;
    vxUp = select(vxSec,componentsVxSec{i}.notTrack); 
    componentsVxSec{i}.directionsAll = PcaLocal(vxUp,6*vxSec.grid,1);
    componentsVxSec{i}.directionsUp  = PcaLocal(vxUp,3*vxUp.grid,1);
    tDirections = toc(tDirections);

    %% Components extraction
    componentsVxSec{i}.masts                = [];
    componentsVxSec{i}.roughMasts           = [];

    componentsVxSec{i}.cables.pairs         = [];
    componentsVxSec{i}.cables.others        = [];

    componentsVxSec{i}.droppers             = [];

    componentsVxSec{i}.signals.big          = [];
    componentsVxSec{i}.signals.trafficLight = [];
    componentsVxSec{i}.signals.stone        = [];
    components.signals.light                = [];
    components.bracketTunnel                = [];
    componentsVxSec{i}.signals.inMast       = [];

    componentsVxSec{i}.rails                = [];        

    %% Mast extraction 
    tMasts             = tic;  
    componentsVxSec{i} = MastsExtraction(vxSec, componentsVxSec{i}, model.mastNoBracketModel);
    tMasts             = toc(tMasts);

    %% Cabling extraction
    tCabling               = tic;
    step                   = 0.2;
    keepingSeeds           = 3*model.mastNoBracketModel.dimensions(1); % 4;
    axis                   = [2:3];
    minLength              = 9;
    marginSearch           = 0.1;

    componentsVxSec{i} = CablesExtraction(vxSec,componentsVxSec{i}, model.cableModel, step, keepingSeeds, axis, minLength, marginSearch);
    tCabling           = toc(tCabling);

    %% Droppers extraction
    tDroppers  = tic;
    distDbscan = 2*vx.grid; % 0.4

    componentsVxSec{i} = DroppersExtraction(vxSec, componentsVxSec{i}, model.dropperModel, distDbscan);
    tDroppers          = toc(tDroppers);

    %% Rails extraction 
    tRails            = tic;
    marginSearch      = 1.5;
    heightFromGround  = 0.1;

    componentsVxSec{i} = RailsExtraction(vxSec, componentsVxSec{i}, model.railPairModel, marginSearch, heightFromGround);
    tRails             = toc(tRails);
    
    %% Signs extraction
    tSigns            = tic;
    percentHighestInt = 0.02;
    inMast            = 1; % range of all voxels close to a mast
    neighborhood      = ceil(gridExtrackTrack/vx.grid) -1; % max neighborhood mistake in pole elements

    componentsVxSec{i} = SignalsExtraction(vxSec, componentsVxSec{i}, percentHighestInt, inMast, model, neighborhood, pcaTraj);
    tSigns             = toc(tSigns);

    %% Plotting in vxSec
    if wantPlotSec
        figure; pcshow(vxSec.Location,'b');
        hold on; pcshow(vxSec.Location(componentsVxSec{i}.track,:),[0.5,0.5,1]);
        title(i)
        if ~isempty(componentsVxSec{i}.masts)
            hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.masts{:}),:), [0.6,0.6,0], 'MarkerSize', 50);
        end
        if ~isempty(componentsVxSec{i}.bracketTunnel)
            hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.bracketTunnel{:}),:), [0.6,0.6,0], 'MarkerSize', 50);
        end
        if ~isempty(componentsVxSec{i}.cables.others)
            hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.cables.others{:}),:), [1,0.5,1], 'MarkerSize', 50);
        end
        if ~isempty(componentsVxSec{i}.cables.pairs)
            value = 0;
            color = [1,0];
            for j = 1:numel(componentsVxSec{i}.cables.pairs)
                hold on; pcshow(vxSec.Location(componentsVxSec{i}.cables.pairs{j}{2},:), [color,0], 'MarkerSize', 50);
                hold on; pcshow(vxSec.Location(componentsVxSec{i}.cables.pairs{j}{1},:), [color/2,1], 'MarkerSize', 50);
                hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.rails{j}{:}),:), [color,0], 'MarkerSize', 50);

                if (numel(componentsVxSec{i}.cables.pairs)-1) ~=0
                    value = j/(numel(componentsVxSec{i}.cables.pairs)-1);
                    if value <0.5
                        color = [1,2*value];
                    else
                        color = [2*(1-value),1];
                    end
                end
            end
        end
        if ~isempty(componentsVxSec{i}.droppers)
            for j = 1:numel(componentsVxSec{i}.droppers)
                if ~isempty(componentsVxSec{i}.droppers{j})
                    aux = cat(1,componentsVxSec{i}.droppers{j}{:});
                    hold on; pcshow(vxSec.Location(aux,:), 'w', 'MarkerSize', 50);
                end
            end
        end
        if ~isempty(componentsVxSec{i}.signals.big)
            hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.signals.big{:}),:), 'y', 'MarkerSize', 50);
        end
        if ~isempty(componentsVxSec{i}.signals.trafficLight)
            hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.signals.trafficLight{:}),:), [1,0.5,0.5], 'MarkerSize', 50);
        end
        if ~isempty(componentsVxSec{i}.signals.light)
            hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.signals.light{:}),:), [0.7,0.7,0.9], 'MarkerSize', 50);
        end
        if ~isempty(componentsVxSec{i}.signals.stone)
            hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.signals.stone{:}),:), [0.7,0.3,0.9], 'MarkerSize', 50);
        end
        if ~isempty(componentsVxSec{i}.signals.inMast)
            hold on; pcshow(vxSec.Location(cat(1,componentsVxSec{i}.signals.inMast{:}),:), 'y', 'MarkerSize', 50);
        end
    end

    %% Index in vx, not in vxSec
    tIndexVx = tic;

    % track, notTrack, roof and wall
    componentsVxSec{i}.track    = idxSection(componentsVxSec{i}.track);
    componentsVxSec{i}.notTrack = idxSection(componentsVxSec{i}.notTrack);
    componentsVxSec{i}.roof     = idxSection(componentsVxSec{i}.roof);
    componentsVxSec{i}.wall     = idxSection(componentsVxSec{i}.wall);

    % Rails
    for j = 1:length(componentsVxSec{i}.rails)
        for k = 1:length(componentsVxSec{i}.rails{j})
            componentsVxSec{i}.rails{j}{k} =  idxSection(componentsVxSec{i}.rails{j}{k}); 
        end
    end

    % Masts
    for j = 1:length(componentsVxSec{i}.masts)
        componentsVxSec{i}.masts{j} = idxSection(componentsVxSec{i}.masts{j});
    end

    % Cables
    for j = 1:length(componentsVxSec{i}.cables.others)
        componentsVxSec{i}.cables.others{j} = idxSection(componentsVxSec{i}.cables.others{j});
    end

    for j = 1:length(componentsVxSec{i}.cables.pairs)
        componentsVxSec{i}.cables.pairs{j}{1} = idxSection(componentsVxSec{i}.cables.pairs{j}{1});
        componentsVxSec{i}.cables.pairs{j}{2} = idxSection(componentsVxSec{i}.cables.pairs{j}{2});
    end

    % Droppers
    for j = 1:length(componentsVxSec{i}.droppers)
        for k = 1:length(componentsVxSec{i}.droppers{j})
            componentsVxSec{i}.droppers{j}{k} = idxSection(componentsVxSec{i}.droppers{j}{k});
        end
    end

    % Signals big
    for j = 1:length(componentsVxSec{i}.signals.big)
        componentsVxSec{i}.signals.big{j} = idxSection(componentsVxSec{i}.signals.big{j});
    end

    % Signals trafficLight
    for j = 1:length(componentsVxSec{i}.signals.trafficLight)
        componentsVxSec{i}.signals.trafficLight{j} = idxSection(componentsVxSec{i}.signals.trafficLight{j});
    end

    % Signals trafficLight
    for j = 1:length(componentsVxSec{i}.signals.light)
        componentsVxSec{i}.signals.light{j} = idxSection(componentsVxSec{i}.signals.light{j});
    end

    % Signals stoneBig
    for j = 1:length(componentsVxSec{i}.signals.stone)
        componentsVxSec{i}.signals.stone{j} = idxSection(componentsVxSec{i}.signals.stone{j});
    end

    % Signals inMast
    for j = 1:length(componentsVxSec{i}.signals.inMast)
        componentsVxSec{i}.signals.inMast{j} = idxSection(componentsVxSec{i}.signals.inMast{j});
    end

    % BracketTunnel
    for j = 1:length(componentsVxSec{i}.bracketTunnel)
        componentsVxSec{i}.bracketTunnel{j} = idxSection(componentsVxSec{i}.bracketTunnel{j});
    end

    tIndexVx = toc(tIndexVx);

   %% Times
   tSection = toc(tSection);

   status.section{i}    = tSection;
   status.selectSection = status.selectSection + tSelectSection;
   status.directions    = status.directions + tDirections;
   status.track         = status.track + tTrack;
   status.rails         = status.rails + tRails;
   status.masts         = status.masts + tMasts;
   status.droppers      = status.droppers + tDroppers;
   status.signs         = status.signs + tSigns;
   status.indexVx       = status.indexVx + tIndexVx;

end

%% Merging components of different sections
status.mergeSections = tic;
componentsVx = [];
% first section
componentsVx.track          = componentsVxSec{1}.track;
componentsVx.notTrack       = componentsVxSec{1}.notTrack;
componentsVx.roof           = componentsVxSec{1}.roof;
componentsVx.wall           = componentsVxSec{1}.wall;

componentsVx.masts          = componentsVxSec{1}.masts;
componentsVx.cables         = componentsVxSec{1}.cables;
componentsVx.droppers       = componentsVxSec{1}.droppers;
componentsVx.signals        = componentsVxSec{1}.signals;
componentsVx.bracketTunnel  = componentsVxSec{1}.bracketTunnel;
componentsVx.rails          = componentsVxSec{1}.rails;

% initializing 
cablesPairsCluster = 1:numel(componentsVx.cables.pairs);
cablesOthersCluster = 1:numel(componentsVx.cables.others);

% Parameters
maxDistX = 3*model.mastNoBracketModel.dimensions(1);
maxDistY = 2;
maxDistZ = 0.3;

% going through all sections adding indixes considering that they can be
% in both sections with ismember()
for i = 2:numel(sections)
    %% track
    element = componentsVxSec{i}.track(~ismember(componentsVxSec{i}.track, componentsVxSec{i-1}.track));
    componentsVx.track = [componentsVx.track; element];

    %% not track
    element = componentsVxSec{i}.notTrack(~ismember(componentsVxSec{i}.notTrack, componentsVxSec{i-1}.notTrack));
    componentsVx.notTrack = [componentsVx.notTrack; element];

    %% roof
    element = componentsVxSec{i}.roof(~ismember(componentsVxSec{i}.roof, componentsVxSec{i-1}.roof));
    componentsVx.roof = [componentsVx.roof; element];

    %% wall
    element = componentsVxSec{i}.wall(~ismember(componentsVxSec{i}.wall, componentsVxSec{i-1}.wall));
    componentsVx.wall = [componentsVx.wall; element];

    %% masts
    if ~isempty(componentsVxSec{i-1}.masts)
        elementsPresection = cat(1, componentsVxSec{i-1}.masts{:});
    else
        elementsPresection = [];
    end
    for j = 1: numel(componentsVxSec{i}.masts)
        if ~any(ismember(componentsVxSec{i}.masts{j},elementsPresection))
            componentsVx.masts{numel(componentsVx.masts)+1} = componentsVxSec{i}.masts{j};
        end
    end

    %% signals big
    if ~isempty(componentsVxSec{i-1}.signals.big)
        elementsPresection = cat(1, componentsVxSec{i-1}.signals.big{:});
    else
        elementsPresection = [];
    end
    for j = 1: numel(componentsVxSec{i}.signals.big)
        if ~any(ismember(componentsVxSec{i}.signals.big{j},elementsPresection))
            componentsVx.signals.big{numel(componentsVx.signals.big)+1} = componentsVxSec{i}.signals.big{j};
        end
    end

    %% signlas traffic light
    if ~isempty(componentsVxSec{i-1}.signals.trafficLight)
        elementsPresection = cat(1, componentsVxSec{i-1}.signals.trafficLight{:});
    else
        elementsPresection = [];
    end
    for j = 1: numel(componentsVxSec{i}.signals.trafficLight)
        if ~any(ismember(componentsVxSec{i}.signals.trafficLight{j},elementsPresection))
            componentsVx.signals.trafficLight{numel(componentsVx.signals.trafficLight)+1} = componentsVxSec{i}.signals.trafficLight{j};
        end
    end

    %% signlas light
    if ~isempty(componentsVxSec{i-1}.signals.light)
        elementsPresection = cat(1, componentsVxSec{i-1}.signals.light{:});
    else
        elementsPresection = [];
    end
    for j = 1: numel(componentsVxSec{i}.signals.light)
        if ~any(ismember(componentsVxSec{i}.signals.light{j},elementsPresection))
            componentsVx.signals.light{numel(componentsVx.signals.light)+1} = componentsVxSec{i}.signals.light{j};
        end
    end

    %% signals stone
    if ~isempty(componentsVxSec{i-1}.signals.stone)
        elementsPresection = cat(1, componentsVxSec{i-1}.signals.stone{:});
    else
        elementsPresection = [];
    end
    for j = 1: numel(componentsVxSec{i}.signals.stone)
        if ~any(ismember(componentsVxSec{i}.signals.stone{j},elementsPresection))
            componentsVx.signals.stone{numel(componentsVx.signals.stone)+1} = componentsVxSec{i}.signals.stone{j};
        end
    end

    %% BracketTunnel
    if ~isempty(componentsVxSec{i-1}.bracketTunnel)
        elementsPresection = cat(1, componentsVxSec{i-1}.bracketTunnel{:});
    else
        elementsPresection = [];
    end
    for j = 1: numel(componentsVxSec{i}.bracketTunnel)
        if ~any(ismember(componentsVxSec{i}.bracketTunnel{j},elementsPresection))
            componentsVx.bracketTunnel{numel(componentsVx.bracketTunnel)+1} = componentsVxSec{i}.bracketTunnel{j};
        end
    end

    %% signals in masts
    if ~isempty(componentsVxSec{i-1}.signals.inMast)
        elementsPresection = cat(1, componentsVxSec{i-1}.signals.inMast{:});
    else
        elementsPresection = [];
    end
    for j = 1: numel(componentsVxSec{i}.signals.inMast)
        if ~any(ismember(componentsVxSec{i}.signals.inMast{j},elementsPresection))
            componentsVx.signals.inMast{numel(componentsVx.signals.inMast)+1} = componentsVxSec{i}.signals.inMast{j};
        end
    end

    %% Linking pairs elements cables
    %% cables pairs, their droppers and their rails
    % If there are not cables in the previous section
    if isempty(componentsVxSec{i-1}.cables.pairs)
        cablesPairsCluster = zeros(length(componentsVxSec{i}.rails),1);
        for j = 1:length(componentsVxSec{i}.rails)
            % Cables
            componentsVx.cables.pairs{numel(componentsVx.cables.pairs)+1} = componentsVxSec{i}.cables.pairs{j};

            % Rails
            componentsVx.rails{numel(componentsVx.rails)+1}               = componentsVxSec{i}.rails{j};

            % Droppers
            componentsVx.droppers{numel(componentsVx.droppers)+1}         = componentsVxSec{i}.droppers{j};

            % Saving the index of this cluster
            cablesPairsCluster(j) = numel(componentsVx.rails);
        end

    % if there are cables in the previous and in this section 
    elseif ~isempty(componentsVxSec{i-1}.cables.pairs) && ~isempty(componentsVxSec{i}.cables.pairs)

        % Updating railsCluster
        newCluster = LinkingBetweenSections(vx.Location,componentsVxSec{i-1}.cables.pairs, componentsVxSec{i}.cables.pairs, traj.points(sections{i}.traj,:), cablesPairsCluster, maxDistX, maxDistY, NaN);

        for j = 1:numel(newCluster)
            % if it is in the previous section
            if newCluster(j) ~=0 
                % Cables
                componentsVx.cables.pairs{newCluster(j)}{1} = [componentsVx.cables.pairs{newCluster(j)}{1}; componentsVxSec{i}.cables.pairs{j}{1}(~ismember(componentsVxSec{i}.cables.pairs{j}{1},componentsVxSec{i-1}.cables.pairs{cablesPairsCluster == newCluster(j)}{1}))];
                componentsVx.cables.pairs{newCluster(j)}{2} = [componentsVx.cables.pairs{newCluster(j)}{2}; componentsVxSec{i}.cables.pairs{j}{2}(~ismember(componentsVxSec{i}.cables.pairs{j}{2},componentsVxSec{i-1}.cables.pairs{cablesPairsCluster == newCluster(j)}{2}))];

                % Rails 
                componentsVx.rails{newCluster(j)}{1} = [componentsVx.rails{newCluster(j)}{1}; componentsVxSec{i}.rails{j}{1}(~ismember(componentsVxSec{i}.rails{j}{1},componentsVxSec{i-1}.rails{cablesPairsCluster == newCluster(j)}{1}))];
                componentsVx.rails{newCluster(j)}{2} = [componentsVx.rails{newCluster(j)}{2}; componentsVxSec{i}.rails{j}{2}(~ismember(componentsVxSec{i}.rails{j}{2},componentsVxSec{i-1}.rails{cablesPairsCluster == newCluster(j)}{2}))];

                % Droppers
                elementsPresection = cat(1,componentsVxSec{i-1}.droppers{cablesPairsCluster == newCluster(j)}{:}); % voxels of all droppers of this pair of cables in the prevoius section
                for k = 1: numel(componentsVxSec{i}.droppers{j})
                    if ~any(ismember(componentsVxSec{i}.droppers{j}{k},elementsPresection))
                       componentsVx.droppers{newCluster(j)}{numel(componentsVx.droppers{newCluster(j)}) + 1} = componentsVxSec{i}.droppers{j}{k};
                    end
                end

            % if it is a new pair of rails  
            else
                % Cables
                componentsVx.cables.pairs{length(componentsVx.cables.pairs) + 1} = componentsVxSec{i}.cables.pairs{j};

                % Rails
                componentsVx.rails{length(componentsVx.rails) + 1} = componentsVxSec{i}.rails{j};

                % Droppers
                componentsVx.droppers{length(componentsVx.droppers) + 1} = componentsVxSec{i}.droppers{j};
                newCluster(j) = length(componentsVx.cables.pairs);
            end
        end
        cablesPairsCluster = newCluster;
    end

    %% cables others
    % If there are not cables in the previous section
    if isempty(componentsVxSec{i-1}.cables.others)
        cablesOthersCluster = zeros(length(componentsVxSec{i}.cables.others),1);
        for j = 1:length(componentsVxSec{i}.cables.others)
            componentsVx.cables.others{numel(componentsVx.cables.others)+1}= componentsVxSec{i}.cables.others{j};
            cablesOthersCluster(j) = numel(componentsVx.cables.others);
        end

    % if there are cables in the previous and in this section 
    elseif ~isempty(componentsVxSec{i-1}.cables.others) && ~isempty(componentsVxSec{i}.cables.others)
        % Updating railsCluster
        newCluster = LinkingBetweenSections(vx.Location,componentsVxSec{i-1}.cables.others, componentsVxSec{i}.cables.others, traj.points(sections{i}.traj,:), cablesOthersCluster, maxDistX, maxDistY, maxDistZ);

        for j = 1:numel(newCluster)
            if newCluster(j) ~=0 % if it is in the previous section
                componentsVx.cables.others{newCluster(j)} = [componentsVx.cables.others{newCluster(j)}; componentsVxSec{i}.cables.others{j}(~ismember(componentsVxSec{i}.cables.others{j},componentsVxSec{i-1}.cables.others{cablesOthersCluster == newCluster(j)}))];
            else % if it is a new pair of rails
                componentsVx.cables.others{length(componentsVx.cables.others) + 1} = componentsVxSec{i}.cables.others{j};
                newCluster(j) = length(componentsVx.cables.others);
            end
        end
        cablesOthersCluster = newCluster;
    end
end

%% Merging rails of diferent contact-cateneary pairs. They are the same rail but they were split because there is a change of contact-catenary wire
if ~isempty(componentsVx.rails)
    [newClusters] = MergingRails(vx.Location, componentsVx.rails, traj); % each componentsVx.rails is assigned to a cluster, merging them if they are the same rail

    % Saving elements that are going to be modified
    componentsAux          = [];
    componentsAux.cables   = componentsVx.cables.pairs;
    componentsAux.droppers = componentsVx.droppers;
    componentsAux.rails    = componentsVx.rails;

    % Deleting element in componentsVx. They will be re-filled up
    componentsVx.cables.pairs = cell(1,length(unique(newClusters)));
    componentsVx.droppers     = cell(1,length(unique(newClusters)));
    componentsVx.rails        = cell(1,length(unique(newClusters)));

    clusters = unique(newClusters); % clusters of final rails
    for i = 1:length(componentsVx.rails)
        componentsVx.rails{i}{1} = [];
        componentsVx.rails{i}{2} = [];


        for j = find(newClusters == clusters(i))' % Adding all the elements of the same cluster

            % Rails
            componentsVx.rails{i}{1} = [componentsVx.rails{i}{1}; componentsAux.rails{j}{1}];
            componentsVx.rails{i}{2} = [componentsVx.rails{i}{2}; componentsAux.rails{j}{2}];

            % Wires. They are assigned to the same cluster but their indexes are not
            % mereged since they are not the same wire, just they are from the same
            % rails
            componentsVx.cables.pairs{i}{length(componentsVx.cables.pairs{i})+1}{1} = componentsAux.cables{j}{1};
            componentsVx.cables.pairs{i}{length(componentsVx.cables.pairs{i})}{2} = componentsAux.cables{j}{2};

            % Droppers as wires
            componentsVx.droppers{i}{length(componentsVx.droppers{i})+1} = componentsAux.droppers{j};
        end
    end
end
status.mergeSections = toc(status.mergeSections);

%% Index in cloud, not in vx
status.cloudIds                 = tic;

components.track                = [];
components.notTrack             = [];
components.wall                 = [];
components.roof                 = [];
components.rails                = [];
components.masts                = [];
components.cables.pairs         = [];
components.cables.others        = []; 
components.droppers             = [];
components.signals.big          = [];
components.signals.trafficLight = [];
components.signals.light        = [];
components.signals.stone        = [];
components.bracketTunnel        = [];
components.signals.inMast       = [];

element = vx.parent_idx(componentsVx.track);
components.track = cat(1,element{:});

element = vx.parent_idx(componentsVx.notTrack);
components.notTrack = cat(1,element{:});

element = vx.parent_idx(componentsVx.wall);
components.wall = cat(1,element{:});

element = vx.parent_idx(componentsVx.roof);
components.roof = cat(1,element{:});

% Rails. Cleaning rails in z
step = 10;
member = false(length(componentsVx.rails));
for j = 1:length(componentsVx.rails) % sorting the pair of rails analysing the number of pair of rails with whom they share points
    for k = 1:length(componentsVx.rails)
        member(j,k) = any(ismember(cat(1,componentsVx.rails{j}{:}),cat(1,componentsVx.rails{k}{:})));
    end
end

[~,order] = sort(sum(member,2),'ascend');
railsIdx = [];
for j = order' % Starting for the pair of rails whcih share points with less pair of rails
    for k = 1:length(componentsVx.rails{j})
        element                          = vx.parent_idx(componentsVx.rails{j}{k});           
        element                          = cat(1,element{:});
        element                          = element(~ismember(element,railsIdx)); % removing indexes of filtered rails considering only Y
        [components.rails{j}{k}, railY ] = DenoisingRails(vx, element, step, model.railModel);
        railsIdx = [railsIdx; railY];
    end
end

% Masts
for j = 1:length(componentsVx.masts)
    element             = vx.parent_idx(componentsVx.masts{j});
    components.masts{j} = cat(1,element{:});
end

% Cables pairs
for j = 1:length(componentsVx.cables.pairs)
    for k = 1: length(componentsVx.cables.pairs{j})
        element                          = vx.parent_idx(componentsVx.cables.pairs{j}{k}{1});
        components.cables.pairs{j}{k}{1} = cat(1,element{:});
        element                          = vx.parent_idx(componentsVx.cables.pairs{j}{k}{2});
        components.cables.pairs{j}{k}{2} = cat(1,element{:});
    end
end

% Cables others
for j = 1:length(componentsVx.cables.others)
    element                     = vx.parent_idx(componentsVx.cables.others{j});
    components.cables.others{j} = cat(1,element{:});
end

% Droppers
for j = 1:length(componentsVx.droppers)
    for k = 1: length(componentsVx.droppers{j})
        for q = 1:length(componentsVx.droppers{j}{k})
            element                      = vx.parent_idx(componentsVx.droppers{j}{k}{q});
            components.droppers{j}{k}{q} = cat(1,element{:});
        end
    end
end

% Signals big
for j = 1:length(componentsVx.signals.big)
    element                   = vx.parent_idx(componentsVx.signals.big{j});
    components.signals.big{j} = cat(1,element{:});
end

% Signals trafficLight
for j = 1:length(componentsVx.signals.trafficLight)
    element                            = vx.parent_idx(componentsVx.signals.trafficLight{j});
    components.signals.trafficLight{j} = cat(1,element{:});
end

% Signals light
for j = 1:length(componentsVx.signals.light)
    element                     = vx.parent_idx(componentsVx.signals.light{j});
    components.signals.light{j} = cat(1,element{:});
end

% Signals stone
for j = 1:length(componentsVx.signals.stone)
    element                     = vx.parent_idx(componentsVx.signals.stone{j});
    components.signals.stone{j} = cat(1,element{:});
end

% BracketTunnel
for j = 1:length(componentsVx.bracketTunnel)
    element                             = vx.parent_idx(componentsVx.bracketTunnel{j});
    components.bracketTunnel{j} = cat(1,element{:});
end

% Signals inMast
for j = 1:length(componentsVx.signals.inMast)
    element                      = vx.parent_idx(componentsVx.signals.inMast{j});
    components.signals.inMast{j} = cat(1,element{:});
end  

%% Plotting in cloud

if wantPlotCloud
    figure; pcshow(vx.parent_cloud,[0.5,0.5,0.5]);
%     hold on; pcshow(vx.parent_cloud(components.track,:),[0.5,0.5,1]);
    if ~isempty(components.masts)
        hold on; pcshow(vx.parent_cloud(cat(1,components.masts{:}),:), [0.6,0.6,0], 'MarkerSize', 50);
    end
    if ~isempty(components.bracketTunnel)
        hold on; pcshow(vx.parent_cloud(cat(1,components.bracketTunnel{:}),:), [0.6,0.6,0], 'MarkerSize', 50);
    end
    if ~isempty(components.cables.others)
        hold on; pcshow(vx.parent_cloud(cat(1,components.cables.others{:}),:), [1,0.5,1], 'MarkerSize', 50);
    end
    if ~isempty(components.cables.pairs)
        value = 0;
        color = [1,0];
        for j = 1:numel(components.cables.pairs)
            hold on; pcshow(vx.parent_cloud(cat(1,components.rails{j}{:}),:), [color,0], 'MarkerSize', 50);
            for k = 1: numel(components.cables.pairs{j})
                hold on; pcshow(vx.parent_cloud(components.cables.pairs{j}{k}{2},:), [color,0], 'MarkerSize', 50);
                hold on; pcshow(vx.parent_cloud(components.cables.pairs{j}{k}{1},:), [color/2,1], 'MarkerSize', 50);
            end

            if (numel(components.cables.pairs)-1) ~=0
                value = j/(numel(components.cables.pairs)-1);
                if value <0.5
                    color = [1,2*value];
                else
                    color = [2*(1-value),1];
                end
            end
        end
    end
    if ~isempty(components.droppers)
        aux = cat(2,components.droppers{:});
        aux = cat(2,aux{:});
        hold on; pcshow(vx.parent_cloud(cat(1,aux{:}),:), [0 0 0], 'MarkerSize', 50);
    end
    if ~isempty(components.signals.big)
        hold on; pcshow(vx.parent_cloud(cat(1,components.signals.big{:}),:), 'y', 'MarkerSize', 50);
    end
    if ~isempty(components.signals.trafficLight)
        hold on; pcshow(vx.parent_cloud(cat(1,components.signals.trafficLight{:}),:), [1,0.5,0.5], 'MarkerSize', 50);
    end
    if ~isempty(components.signals.light)
        hold on; pcshow(vx.parent_cloud(cat(1,components.signals.light{:}),:), [0.7,0.7,0.9], 'MarkerSize', 50);
    end
    if ~isempty(components.signals.stone)
        hold on; pcshow(vx.parent_cloud(cat(1,components.signals.stone{:}),:), [0.7,0.3,0.9], 'MarkerSize', 50);
    end
    if ~isempty(components.signals.inMast)
        hold on; pcshow(vx.parent_cloud(cat(1,components.signals.inMast{:}),:), 'y', 'MarkerSize', 50);
    end
    WhitePcshow()
end

%% Indexes in the original cloud, not in the input cloud

components.track    = idxOriginal(components.track);
components.notTrack = idxOriginal(components.notTrack);
components.wall     = idxOriginal(components.wall);
components.roof     = idxOriginal(components.roof);

% Rails
for j = 1:length(componentsVx.rails)
    for k = 1:length(componentsVx.rails{j})
        components.rails{j}{k} = idxOriginal(components.rails{j}{k});
    end
end

% Masts
for j = 1:length(componentsVx.masts)
    components.masts{j} = idxOriginal(components.masts{j});
end

% Cables pairs
for j = 1:length(componentsVx.cables.pairs)
    for k = 1: length(componentsVx.cables.pairs{j})
        for q = 1:length(componentsVx.cables.pairs{j}{k})
            components.cables.pairs{j}{k}{q} = idxOriginal(components.cables.pairs{j}{k}{q});
        end
    end
end

% Cables others
for j = 1:length(componentsVx.cables.others)
    components.cables.others{j} = idxOriginal(components.cables.others{j});
end

% Droppers
for j = 1:length(componentsVx.droppers)
    for k = 1: length(componentsVx.droppers{j})
        for q = 1: length(componentsVx.droppers{j}{k})
            components.droppers{j}{k}{q} = idxOriginal(components.droppers{j}{k}{q});
        end
    end
end

% Signals big
for j = 1:length(componentsVx.signals.big)
    components.signals.big{j} = idxOriginal(components.signals.big{j});
end

% Signals trafficLight
for j = 1:length(componentsVx.signals.trafficLight)
    components.signals.trafficLight{j} = idxOriginal(components.signals.trafficLight{j});
end

% Signals light
for j = 1:length(componentsVx.signals.light)
    components.signals.light{j} = idxOriginal(components.signals.light{j});
end

% Signals stone
for j = 1:length(componentsVx.signals.stone)
    components.signals.stone{j} = idxOriginal(components.signals.stone{j});
end

% BracketTunnel
for j = 1:length(componentsVx.bracketTunnel)
    components.bracketTunnel{j} = idxOriginal(components.bracketTunnel{j});
end

% Signals inMast
for j = 1:length(componentsVx.signals.inMast)
    components.signals.inMast{j} = idxOriginal(components.signals.inMast{j});
end

% indexes in the orginal that have been evaluated (cloud input == vx)
components.evaluated = idxOriginal;

status.cloudIds  = toc(status.cloudIds);
end


