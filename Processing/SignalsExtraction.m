function [components] = SignalsExtraction(vx, components, percentHighestInt, inMast, model, neighborhood, pcaTraj)
% Signals extraction
%
% Signs can be in mast or can be an independent element. Signals in mast
% are extracted by intensity and the indepent signals by geometry.
%
% Signals in mast. Each mast is observed looking for signal. That sings
% have points with a notorius high intensity. The mean position in XY of a 
% mast is calculated. All voxels in XY with a distance lower than inMast
% are selected, but only the ones in notTrack and not in wall. The
% intensity of these points is normalized and an  intensity histogram is 
% mame. If the % of points in the column of the highest intenisty is higher
% than percentHighestInt might be a signal. In that case, points with a
% intensity higher than the first local min are clustered. Clusters with
% enough points are considered signals and its neighbours too.
%
% Signals elements extraction. Voxels in notTrack not yet classified and 
% its down neighbours are clustered. Down voxels are added to complete its
% pole since it might be classified as track (1 voxel in track extraction
% error). Clusters with enough elements and without any voxel in wall2 are 
% compared with signals models using CheckElement(). To make the
% comparation it is used the original points to have more resolution. Those
% points are oriented with the trajectory and centered considering its
% highest part, avoiding problems with pole length (model clouds have the 
% the same orientation). If the result of the comparation is a true, the 
% cluster is saved as that sign.
%
%--------------------------------------------------------------------------
% INPUTS:
%
% vx : Voxels. Cloud 
% 
% components : cell. Cell with elements extracted
% 
% percentHighestInt : numeric. Min points with high intensity in a mast to
%                     consider a signal in it on for one unit.
%
% inMast : numeric. distance of points close to a mast that are considered. 
%
% ---Model : Element. Model of each element
%
% neighborhood : numeric. levels of neighborhood to retrieve from the
%                ground to not cut poles.
%
%--------------------------------------------------------------------------
% OUTPUTS:
%
% components.signals.big : cell 1 x num of big signal. Each cell has a cell
%                          with the indexes of a big signal.
%
% components.signals.trafficLight : cell 1 x num of big signal. Each cell 
%                                   has a cell has with the indexes of a 
%                                   trafficLight.
% 
% components.signals.stone : cell 1 x num of stone signal. Each cell has a 
%                            cell has with the indexes of a signal of 
%                            stone.
%
% components.signals.light : cell 1 x num of light signal. Each cell has a 
%                            cell has with the indexes of a light signal.
%
%--------------------------------------------------------------------------
% Daniel Lamas Novoa
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 23/12/2020

%% Signals in masts
components.signals.inMast = [];
for i = 1:numel(components.roughMasts)
    mast = components.notTrack(sqrt((vx.Location(components.notTrack,1)- mean(vx.Location(components.roughMasts{i},1))).^2  + (vx.Location(components.notTrack,2)- mean(vx.Location(components.roughMasts{i},2))).^2) < inMast); % All voxels arounf this mast
    mast = mast(~ismember(mast,components.wall)); % Denoising. Deleting wall voxels
    
%     figure; pcshow(vx.Location(mast,:),'r', 'MarkerSize', 50);
%     hold on; pcshow(vx.Location(components.roughMasts{i},:),'g', 'MarkerSize', 50);
    
    intensity = (vx.intensity(mast,:) - min(vx.intensity(mast,:))) / max(vx.intensity(mast,:)); % normalized intensity
    [num, edges] = histcounts(intensity,20); % histogram
%     figure;histogram(intensity, 20)
    
    if num(end)/sum(num) > percentHighestInt % if the percentil with most intensity has more points than % of total points
        localMin = edges(find(islocalmin(num), 1, 'first')); % intensity of the first local minimum
        signals = mast(intensity > localMin); % points with high intensity
        
        % Clustering
        idx = dbscan(vx.Location(signals,:), 1.1*vx.grid,1);
        num = groupcounts(idx); 
        [num, clusters] = sort(num,'descend');
        clusters = clusters(num > 20); % Deleting noise
        signals = signals(ismember(idx, clusters));
        
        % Adding neighbours        
        neighbours = vx.neighbours_rows(signals,:);
        neighbours = reshape(neighbours,numel(neighbours),1);
        neighbours = neighbours(~isnan(neighbours));
        neighbours = neighbours(ismember(neighbours, components.notTrack) & ~ismember(neighbours, components.wall)); % in notTrack and not in wall
        signals = [signals; neighbours];
        components.signals.inMast{numel(components.signals.inMast) + 1} = signals;
         
%         figure; pcshow(vx.Location(mast,:), 'b', 'MarkerSize',50);
%         hold on; pcshow(vx.Location(signals,:), 'y', 'MarkerSize',50);         
%         j = 0;
%         j = j+1
%         hold on; pcshow(vx.Location(signals(ismember(idx, clusters(j))),:), 'y', 'MarkerSize',50);
   end    
end

%% Working with notTrack cloud and deleting masts, droppers and cables.

components.signals.big           = [];
components.signals.trafficLight  = [];
components.signals.stone         = [];
components.signals.light         = [];
components.bracketTunnel         = [];

elements = [];
if ~isempty(components.masts)
    elements = [elements; cat(1,components.masts{:})];
end
if ~isempty(components.cables.pairs)
    aux = cat(2,components.cables.pairs{:});
    aux = cat(1,aux{:});
    elements = [elements; aux];
end
if ~isempty(components.cables.others)
    elements = [elements; cat(1,components.cables.others{:})];
end
if ~isempty(components.droppers)
    aux = cat(2,components.droppers{:});
    if ~isempty(aux)
        aux = cat(1,aux{:});
        elements = [elements; aux];
    end
end

% Only voxels in notTrack and deleting elements already extracted and wall
possibleSignals = components.notTrack(~ismember(components.notTrack,elements)); % & ~ismember(components.notTrack,components.wall));

% Adding down neighbours just in case there are some cutted poles
for i = 1:neighborhood
    possibleSignalsToo = vx.neighbours_rows(possibleSignals,5);
    possibleSignalsToo = reshape(possibleSignalsToo,numel(possibleSignalsToo),1);
    possibleSignalsToo = possibleSignalsToo(~isnan(possibleSignalsToo));
    possibleSignalsToo = possibleSignalsToo(~ismember(possibleSignalsToo,elements)); % & ~ismember(possibleSignalsToo,components.wall)); % Deleting elements
    possibleSignals    = unique([possibleSignals; possibleSignalsToo]);
end

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(possibleSignals,:), 'w', 'MarkerSize',50);
% hold on; pcshow(vx.Location(components.wall2,:), 'y', 'MarkerSize',50);


if isempty(possibleSignals)
    return;
end

%% Clustering
vxSignals = select(vx,possibleSignals);
idx = ClusteringNeighbours(vxSignals, 'distance', 1.3*vx.grid);
[num] = groupcounts(idx); 
[num, clusters] = sort(num,'descend');
clusters = clusters(num > 2/vx.grid); % 20

% color = [rand(1), rand(1),0];
% i = i+1
% hold on; pcshow(vx.Location(possibleSignals(idx == clusters(i)),:),color, 'MarkerSize',50);

    
%% Checking each cluster

for i = 1:numel(clusters)
    
%     hold on; pcshow(vx.Location(possibleSignals(idx == clusters(i)),:),'r', 'MarkerSize',50);
     
    if all(~ismember(possibleSignals(idx == clusters(i)), components.wall2)) % Without any voxel in wall2 
                       
        % element in parent_cloud. It has more resolution
        % Centered on its highest part
        element = vx.parent_idx(possibleSignals(idx == clusters(i)));
        element = vx.parent_cloud(cat(1,element{:}),:);
        element = element * pcaTraj;
        element = element - mean(element);
        element(:,3) = element(:,3) - max(element(:,3));
%         figure; pcshow(element,'MarkerSize', 100);

        if CheckElement(element, model.bigSignalModel)
             components.signals.big{numel(components.signals.big) + 1} = possibleSignals(idx == clusters(i));

        elseif CheckElement(element, model.trafficLightModel) || CheckElement(element, model.trafficLightModel2) || CheckElement(element, model.trafficLightModel3) 
            components.signals.trafficLight{numel(components.signals.trafficLight) + 1} = possibleSignals(idx == clusters(i));

        elseif CheckElement(element, model.stoneSignalModel) || CheckElement(element, model.stoneSignalModel3)
            components.signals.stone{numel(components.signals.stone) + 1} = possibleSignals(idx == clusters(i));
            
        elseif CheckElement(element, model.light)
            components.signals.light{numel(components.signals.light) + 1} = possibleSignals(idx == clusters(i));
        end
    end
end
end
% CODE TO SAVE AN ELEMENT MODEL
%% Element that will be used as model using cloud point, not voxels
% model= vx.parent_idx(possibleSignals(idx == clusters(i)));
% model = cat(1,model{:});        
% model = vx.parent_cloud(model,:);
% model = model - mean(model);
% model = model * pca(model); % orineting and centering
% % dim 1 is the original Z because it is a signal.
% % Centering by max Z because not all times the signal will be recognized in
% % the same level of the pole.
% model(:,1) = model(:,1) - max(model(:,1));
% %         
% % figure; pcshow(model, 'MarkerSize', 50);
% %         
% figure; pcshow(a, 'MarkerSize', 50)
% element = element(element(:,3) > -1,:);
% figure; pcshow(points, 'MarkerSize', 50)
% trafficLightSigns3 = points;
% save(['C:\Users\danie\Desktop\SAFEWAY2020\Segmentacion\Ferrocarril\Models\New', '\','trafficLightSigns3.mat'], 'trafficLightSigns3');
% % 
% % Parameters
% model2 = vx.Location(possibleSignals(idx == clusters(i)),:);
% figure; pcshow(model2, 'MarkerSize', 50);
% [autvec, lat] = pcacov(cov(model2));
% for i = 1:3
%     autval(i) = lat(i)/sum(lat);
% end

% figure; pcshow(vx.Location, 'b', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1, components.signals.big{:}),:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1, components.signals.trafficLight{:}),:), 'g', 'MarkerSize',50);
% hold on; pcshow(vx.Location(cat(1, components.signals.stone{:}),:), 'r', 'MarkerSize',50);
% WhitePcshow;


