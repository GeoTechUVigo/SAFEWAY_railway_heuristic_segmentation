function [newClusters] = LinkingBetweenSections(vxLocation, elementsPre, elementsSec, trajPoints, clusters, maxDistX, maxDistY, maxDistZ)
% This function analyzes all the clusters of elementsPre, looking for if
% there is any cluster of elementsSec that is the continuation of it. If
% they are the same element, the cluster in elementsSec will have the same
% number in newCluster that the cluster in elementsSec has in clusters.
% if not it will have a new number because it is a new element.
%
% Solving problems if it is not a pair of elements. elementsSec can have
% pairs (or more) of elements or indiviual elements. If they are individual
% element it is need to transform it into a cell array to be consistan with
% the code.
%
% The variable elements have the indexes of all the points of elementsSec
% and elementsPre in vxLocation. element{1} has the indexes of 
% elementsPre{1}, and element{num elements pre + 1} has the indexes of
% elementsSec{1}. elementsPre{i}{1} and elementsPre{i}{2} are the pair of
% the element i. At the same time, the varialb idx is created. It is an
% array with a different index to each cluster in elements{}.
% 
% The elements are oriented by trajPoints and its location is saved in the
% variable location{} usinng elements{}. With idx and location{} it is
% possible to identify each element oriented, being the first the elements
% in previous section and the lasts the elements in this sec.
%
% Going over elementsPre looking for its continuation. Each element in the
% previous section is analyzed with all the elements in the next section.
% First, common points are calculated (might be some common points if
% there are overlap between sections). After, the end of the element in pre
% and the start of the next elements in the section are compared, but
% avoiding overlap, so choosing the start of the element as the first point
% with higher X than the end of the element in previous section. This
% distance is measured in X and in Y. Clusters with the start part in
% range (close enough) are analyzed. The one with more common points is the
% continuation. If there are a tie, the one with less Y distance is the
% continuation.
%
% Undoing ties. Might be some clusters in previous section classified with
% the same cluster continuation. The good one is the one with less distance
% in Y to its continuation. The others do not have continuation.
%
% This analysis is done taking into account just elements of the same type
% In the case of cables, location{1} and location{2} independently, being 1
% catenary cables and 2 contact cables. However, both must have the same
% continuation pair, so if at least one have a continuation, the other has
% its pair.
%
%--------------------------------------------------------------------------
% INPUTS:
%
% vxLocation : Nx3. Cloud.Location 
% 
% elementsPre : cell. Cell with elements in previous section
% 
% elementsSec : cell. Cell with elements in this section
% 
% trajPoints : Nx3. trajectory.points
%
% clusters : num of clusters in previous section x 1. It has the indexes of
%            the cluster of the elements in the previous section.
%
% maxDistX : numeric. Max distance in X axis between the end of an element
%            and the start of its continuation.
%
% maxDistY : numeric. Max distance in Y axis between the end of an element
%            and the start of its continuation.
%                
% -------------------------------------------------------------------------
% OUTPUTS:
% 
% newClusters : num of clusters in this section x 1. It has the indexes of
%               the cluster of the elements in this section. Elements that
%               are the continuation of any one will have the same number
%               that its element in previous section has in the variable
%               cluster. newClusters(i) = clusters(j) if elementsSec{i} is
%               the continuation of elementsPre{j}. Elements that do not
%               have previous cluster have a 0.
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 23/12/2020

%% Solving problems if it is not a pair of elements
if ~iscell(elementsSec{1})
    aux = cell(1,numel(elementsSec));
    for i = 1:numel(elementsSec)
        aux{i}{1} = elementsSec{i};
    end
    elementsSec = aux;
    
    aux = cell(1,numel(elementsPre));
    for i = 1:numel(elementsPre)
        aux{i}{1} = elementsPre{i};
    end
    elementsPre = aux;  
end

%%
elements = cell(1, numel(elementsSec{1}));
idx = cell(1, numel(elementsSec{1}));

%% Voxels of elements that are from this section
for i = 1:numel(elementsPre)
    for j = 1: numel(elementsSec{1})
        elementIndexes = elementsPre{i}{j};
        elements{j} = [elements{j}; elementIndexes];
        aux = zeros(numel(elementIndexes),1);
        aux(:) = i;
        idx{j} = [idx{j};aux];
    end
end

%% Voxels of elements of the next section
for i = 1:numel(elementsSec)
    for j = 1:numel(elementsSec{1})
        elements{j} = [elements{j}; elementsSec{i}{j}];
        aux = zeros(numel(elementsSec{i}{j}),1);
        aux(:) = i + numel(elementsPre);
        idx{j} = [idx{j};aux];
    end
end

% figure; pcshow(vxLocation,'b')
% hold on; pcshow(vxLocation(elements{2},:),'y', 'MarkerSize',100)
% hold on; pcshow(vxLocation(elements{1},:),'r', 'MarkerSize',100)
% hold on; pcshow(vxLocation(elements{2}(idx{2} == 4),:),'g', 'MarkerSize',100)

%% Orienting cloud by traj
trajOriented = trajPoints - mean(trajPoints(:,:));
pcaTraj      = PcaFlattering(trajPoints); % Appling pca to the traj
trajOriented = trajOriented * pcaTraj; % Oriented traj points with its pca
% Checking if trajPoinjt is in the the correct order
if trajOriented(end,1) < trajOriented(1,1)
    rot180Pca3   = makehgtform('axisrotate',pcaTraj(:,3),180*pi/180); % rotate 180 degrees in pca(3), that is the new z 4x4 matrix
    rot180Pca3   = rot180Pca3(1:3,1:3); % 3x3 matrix
    pcaTraj      = pcaTraj * rot180Pca3;
    trajOriented = trajPoints - mean(trajPoints(:,:));
    trajOriented = trajOriented * pcaTraj;
end

for i = 1: numel(elementsSec{1})
    location{i} = vxLocation(elements{i},:) - mean(trajPoints(:,:));
    location{i} = location{i} * pcaTraj;
end

% figure; pcshow(trajOriented,'y', 'MarkerSize',100)
% hold on; pcshow(location{1},'g', 'MarkerSize', 200)
% hold on; pcshow(location{2},'r', 'MarkerSize', 200)
% hold on;pcshow(location{1}(idx{1} == 2,:),'y', 'MarkerSize', 200)

%% Going over elementsPre looking for its continuation
% figure; pcshow(location{1},'b', 'MarkerSize',100)
% hold on; pcshow(location{2},'b', 'MarkerSize',100)

nextCluster = zeros(numel(elementsPre),1);
distances = zeros(numel(elementsPre),1); % Saving the distance between cables linked because might be more than one in range
for i = 1:numel(elementsPre)
    
    % Calculing common points in cluster and distances deleting overlap
    commonPoints = zeros(numel(elementsSec),numel(elementsPre{1}));
    distZ        = zeros(numel(elementsSec),numel(elementsPre{1}));
    distY        = zeros(numel(elementsSec),numel(elementsPre{1}));
    distX        = zeros(numel(elementsSec),numel(elementsPre{1}));
    for j = 1:numel(elementsPre{1})
        
        % Considering overlap
        for k = 1: numel(elementsSec)
            commonPoints(k,j) = sum(ismember(elementsSec{k}{j},elementsPre{i}{j})); % points in common because of overlap
        end
    
        % Distances between the end of the element in pre sections and the first points of all
        % the elementes in section with a X > than the end of the element
        % pre 
        elementEnd = location{j}(location{j}(:,1) == max(location{j}(idx{j} == i,1)), :);
        elementEnd = elementEnd(1,:);
        for k = 1: numel(elementsSec)
            elementStart = location{j}(idx{j} == k + numel(elementsPre),:); %% all the element
            elementStart = elementStart(elementStart(:,1) > elementEnd(1),:); % element without overlap
            elementStart = elementStart(elementStart(:,1) == min(elementStart(:,1)), :); % start of this element
            
            % if isempty it means that the cluster ends before the end of the section, so to make the comparation is chosen the last point
            % if they are the same cluster, both end points should be the
            % same
            if isempty(elementStart)
                elementStart = location{j}(location{j}(:,1) == max(location{j}(idx{j} == k + numel(elementsPre),1)), :);
            end
                
            elementStart = elementStart(1,:);
            distX(k,j)   = abs(elementStart(1) - elementEnd(1));
            distY(k,j)   = abs(elementStart(2) - elementEnd(2));
            distZ(k,j)   = abs(elementStart(3) - elementEnd(3));
        end
    end
    
    distY(distX > maxDistX | distY > maxDistY | distZ > maxDistZ) = NaN; % NaN if they are out of range
    
    % Chossing the index of the cluster that is the continuation of this
    if any(~isnan(distY)) % if any element in section are in range
        % Considering only elements in range in distances
        commonPoints(isnan(distY)) = NaN;
        if any(any(commonPoints)) % the element is the one with more common points
            [~,nextCluster(i)] = max(max(commonPoints,[],2));
            distances(i) = min(distY(nextCluster(i),:));
        else % if there are no common points, it is the one with less Y distance
            [distances(i),nextCluster(i)] = min(min(distY,[],2));
        end
    end

%     hold on; pcshow(location{j}(idx{j} == i,:),'w', 'MarkerSize',100)
%     hold on; pcshow(location{1}(idx{1} == 2 + numel(elementsPre),:),'r', 'MarkerSize',100)
% 
%     hold on; pcshow(elementEnd,'w','MarkerSize',2000)
%     hold on; pcshow(elementStart,'g','MarkerSize',2000)
end

%% Undoing ties 
% More than one element has the same continuation element (same number in nextCluster) The good is the one with less distance
for i = 1:numel(nextCluster)
    if sum(nextCluster(i) == nextCluster) > 1 % More than itselft cluster pre has the same cluster continuation
        a = find(nextCluster(i) == nextCluster); % Clusters in pre with this continuation (nexCluster(i))
        [~,b] = min(distances(a));
        b = a(b); % Cluster in pre that is the real continuation of this cluster
        nextCluster(a(~ismember(a,b))) = 0; % removing the number of cluster errors
    end
end
%% If at least one element of the couple has a previous element, both have it
newClusters = zeros(length(elementsSec),1);
for i = 1:length(clusters)
        if nextCluster(i) ~=0
            newClusters(nextCluster(i)) = clusters(i);
        end
end

% figure; pcshow(vxLocation,'b')
% hold on; pcshow(vxLocation(elementsPre{4}{1},:),'r')
% hold on; pcshow(vxLocation(elementsSec{newClusters == 2}{1},:),'g')
% 
% hold on; pcshow(vxLocation(elementsSec{11}{1},:),'r')
% 
% hold on; pcshow(vxLocation(elementsPre{1}{2},:),'g')
% hold on; pcshow(vxLocation(elementsSec{11}{2},:),'g')

end

