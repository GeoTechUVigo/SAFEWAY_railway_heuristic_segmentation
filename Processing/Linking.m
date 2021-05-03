function [idx] = Linking(vx, peaks, step, widthElement, keepingSeeds, axis)
% Peaks from the same cluster have the same idx number.
% In the first section, peaks that are together have the same idx and only
% 1 is the seed of the cluster. Seeds alone have its unique idx and they 
% are seeds.
%
% Each cluster has a unique seed. It seed is a peak inside the range of the
% previous seed and the one with the Y most similar to the mean of the
% cluster.
%
% In the nexts sections, each new peak is asigned to a cluster.
%
% If the new peak is inside the limits of one seed it will be from the same
% cluster. If it inside the limits of more than one seed, it is from the
% most similar position in Y. If it is the most similar of the mean of the
% peaks added, it is the next seed.
%
% If the peak has not a seed, it makes a new cluster.
%
% Each seed with no next peaks are kept like a seed only some sections.
%
% The similitude between peaks and seeds are evaluated in the axis (axis).
% 
%--------------------------------------------------------------------------
% INPUTS:
% 
% vx : Voxels. Cloud 
% 
% peaks : Nx1 numeric. Indexes of elements in vx to be linked.
%
% step : numeric. Width of sections.
%
% widthElement : numeric. Width of the element to be linked.
%
% keepingSeeds : numeric. Distances that a seed without continuation is
%                used to look for its continuation.
%
% axis : N numeric. Dimensions used to measured distances between points.
%
% -------------------------------------------------------------------------
% OUTPUTS:
% 
% idx : Nx1. Indexes that indicates the cluster of each peak.
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 23/12/2020

%% Linking peaks
idx     = zeros(size(peaks)); % idx of the peaks
maxDist = max(vx.Location(:,1));
minDist = min(vx.Location(:,1));

% peaks in first section
minimum   = minDist;
maximum   = minDist + step;
nextPeaks = find(vx.Location(peaks,1) >= minimum & vx.Location(peaks,1) < maximum);

% Clustering nextpeaks in first section, assigning a nextSeed and nextMeanCluster for each cluster
% Saving with the same idx all the voxels in the same cluster
lastNum = 0;
if ~isempty(nextPeaks)
    
    idxFirstSection = dbscan(vx.Location(peaks(nextPeaks), axis), widthElement/2, 1);
    clusters        = unique(idxFirstSection);
    nextSeeds       = zeros(size(clusters));
    nextMeanCluster = zeros(numel(clusters),numel(axis));
    nextBigCluster   = false(size(clusters));
    
    for i = 1: numel(clusters)
        lastNum                              = lastNum + 1;
        idx(nextPeaks(idxFirstSection == i)) = lastNum;
        nextSeeds(i)                         = nextPeaks(find(idxFirstSection == i,1,'first'));
        nextMeanCluster(i,:)                 = vx.Location(peaks(nextSeeds(i)),axis);
    end
    
else
    
    nextSeeds       = [];
    nextMeanCluster = [];
    nextBigCluster   = [];
    
end

% Going throgh all peaks section by section
for i = minDist:step:maxDist

    % Updating section with the peaks choosen deleting seeds that are
    % far behind, and saving thier mean in axis   
    seeds       = nextSeeds(vx.Location(peaks(nextSeeds),1) > minimum - keepingSeeds);
    meanCluster = nextMeanCluster(vx.Location(peaks(nextSeeds),1) > minimum - keepingSeeds,:);
    bigCluster  = nextBigCluster(vx.Location(peaks(nextSeeds),1) > minimum - keepingSeeds);
    
    % peaks in next section
    maximum   = i + step;
    minimum   = i;
    nextPeaks = find(vx.Location(peaks,1) >= minimum & vx.Location(peaks,1) < maximum);
    
    % Reseting
    nextSeeds       = [];
    nextMeanCluster = [];
    nextBigCluster  = [];
    visited         = false(size(nextPeaks)); % To save peaks that have a seed close enough
    keep            = true(size(seeds)); % to keep seeds that have not got next tip close enough
    for j = seeds'
        
        inRange = find(pdist2(vx.Location(peaks(nextPeaks),axis), vx.Location(peaks(j),axis)) < widthElement); % Next peaks inside the limits in axis 
        
        if any(inRange)
            
            visited(inRange) = true; % Saving peaks that have a seed close enough
            distances        = pdist2(vx.Location(peaks(nextPeaks(inRange)),axis), meanCluster(seeds == j,:)); % distances to mean of axis
            [~, nextSeed]    = min(distances); % the most similar y in range could be the next seed
            nextSeed         = inRange(nextSeed);
            
            for k = inRange' % Going through all peaks in range

                % If it could be the nextSeed
                if k == nextSeed 
            
                    % The tip has not got cluster
                    if idx(nextPeaks(k)) == 0 
                        
                        keep(seeds == j)  = false; % this seed has a next seed
                        idx(nextPeaks(k)) = idx(j); % Saving the idx of the cluster
                        nextSeeds         = [nextSeeds; nextPeaks(k)]; % Adding the seed of the cluster
                        nextMeanCluster   = [nextMeanCluster; mean([vx.Location(peaks(nextPeaks(k)),axis);meanCluster(seeds == j,:);meanCluster(seeds == j,:)])]; % Adding the mean of axis
                        
                    % The tip is for any cluster, so we have to choose its best cluster
                    elseif (bigCluster(seeds == j) == bigCluster(idx(seeds) == idx(nextPeaks(k))) &&... % Or both big cluster or small
                            distances(inRange == k) < pdist2(vx.Location(peaks(nextPeaks(k)),axis), meanCluster(idx(seeds) == idx(nextPeaks(k)),:))) ||... % if it is the closets one
                            (bigCluster(seeds == j) == true && bigCluster(idx(seeds) == idx(nextPeaks(k))) == false) % The new is big and the old small
                        
                        % it would be a seed of other cluster
                        if ismember(nextPeaks(k),nextSeeds) 
                            
                            keep(seeds == j)                             = false; % this seed has a next seed
                            keep(idx(seeds) == idx(nextPeaks(k)))        = true; % this seed has not got a next seed
                            idx(nextPeaks(k))                            = idx(j); % Changing the idx of the cluster
                            nextMeanCluster(nextSeeds == nextPeaks(k),:) = mean([vx.Location(peaks(nextPeaks(k)),axis);meanCluster(seeds == j,:);meanCluster(seeds == j,:)]); % Changing the mean of axis                
                            
                        else
                            
                            keep(seeds == j)  = false; % this seed has a next seed
                            idx(nextPeaks(k)) = idx(j); % Changing the idx of the cluster
                            nextSeeds         = [nextSeeds; nextPeaks(k)]; % Adding the seed of the cluster
                            nextMeanCluster   = [nextMeanCluster; mean([vx.Location(peaks(nextPeaks(k)),axis);meanCluster(seeds == j,:);meanCluster(seeds == j,:)])]; % Adding the mean of axis
                            
                        end                   
                    end
                    
                % Same but not adding this tip in nextSeeds    
                else 
                    
                    % The tip has not got cluster
                    if idx(nextPeaks(k)) == 0 
                        
                        idx(nextPeaks(k)) = idx(j); % Saving the idx of the cluster
                        
                    % The tip is for any cluster, so we have to choose its best cluster
                    elseif (bigCluster(seeds == j) == bigCluster(idx(seeds) == idx(nextPeaks(k))) &&... % Or both big cluster or small
                            distances(inRange == k) < pdist2(vx.Location(peaks(nextPeaks(k)),axis), meanCluster(idx(seeds) == idx(nextPeaks(k)),:))) ||... % if it is the closets one
                            (bigCluster(seeds == j) == true && bigCluster(idx(seeds) == idx(nextPeaks(k))) == false) % The new is big and the old small
                        
                        % it would be a seed of other cluster
                        if ismember(nextPeaks(k),nextSeeds)
                            
                            keep(idx(seeds) == idx(nextPeaks(k)))        = true; % this seed has not got a next seed
                            idx(nextPeaks(k))                            = idx(j); % Changing the idx of the cluster
                            nextMeanCluster(nextSeeds == nextPeaks(k),:) = []; % Deleting mean
                            nextSeeds(nextSeeds == nextPeaks(k))         = []; % Deleting next seed
                            
                
                        else
                            
                            idx(nextPeaks(k)) = idx(j); % Changing the idx of the cluster
                            
                        end                                        
                    end                    
                end
            end
        end
    end
    
    % Generating new clusters with the seeds that have not been visited as
    % in the first section, assigning same idx for seeds that are together
    % and chossing only 1 seeds for cluster
    nextPeaksNoVisited = nextPeaks(~visited); % nextpeaks without being assigned to a cluster 
    
    if ~isempty(nextPeaksNoVisited) % if there is any
        
        idxNoVisited = dbscan(vx.Location(peaks(nextPeaksNoVisited), axis), widthElement/2, 1); % clustering
        clusters     = unique(idxNoVisited);

        for j = 1: numel(clusters) % going throug all clusters
            
            lastNum                                    = lastNum + 1;
            idx(nextPeaksNoVisited(idxNoVisited == j)) = lastNum; % Saving idx
            nextSeeds                                  = [nextSeeds; nextPeaksNoVisited(find(idxNoVisited == j,1,'first'))]; % new seed
            nextMeanCluster                            = [nextMeanCluster; vx.Location(peaks(nextSeeds(end)),axis)];
            
        end  
    end
    
    % Analysing if it is a big cluster or not
    nextBigCluster = false(size(nextSeeds));
    for j = 1:numel(nextSeeds)
        if range(idx == idx(nextSeeds(j))) > step
            nextBigCluster(j) = true;
        end
    end
    
    % Keeping seeds that have not got a next seed
    nextSeeds       = [nextSeeds; seeds(keep)];
    nextMeanCluster = [nextMeanCluster; meanCluster(keep,:)];
    nextBigCluster  = [nextBigCluster; bigCluster(keep)];
    
end

% figure; pcshow(vx.Location, 'b')
% hold on; pcshow(vx.Location(peaks(seeds),:), 'r', 'MarkerSize', 50)
% hold on; pcshow(vx.Location(peaks(nextSeeds),:), 'g', 'MarkerSize', 50)

end

