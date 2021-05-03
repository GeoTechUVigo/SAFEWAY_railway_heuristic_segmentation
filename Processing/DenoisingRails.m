function [rail, railY] = DenoisingRails(vx, railIdx, step, railModel)
% This function "clean" the rails deleting points from the track.
%
% The rail is oriented by its own pca resultf.
%
% The rail is splitted in sections by X direction.
%
% Each section is oriented. An histogram of Y is done and points close to
% the Y value with more points are selected.
% 
% The mean in Z of the selected points is calculated. Points with higher Z
% than the mean are classified as rail.
%
%--------------------------------------------------------------------------
% INPUTS:
%
% vx : Voxels.  
%
% railIdx : Nx1 numeric. Indexes of this rail.
%
% step : numeric. Width of the sections to analyze the rail.
%
% railModel : Element. Rail's model
%
% -------------------------------------------------------------------------
% OUTPUTS:
% 
% rail : Nx1 numeric. Indexes of the rail filtered.
% 
% railY : Nx1 numeric. Indexes of the rail ocnsidering only Y. This indexes
%         are not considered in futures rails 
%
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 23/12/2020
%
%--------------------------------------------------------------------------
% Posible mejora: quitar ruida al rail al principio con la función de
% matlab y después en vez de coger por mean(z) coger por max(z)- alto del
% rail
% Posible mejora: hacer un código especial para cuando hay un crice re
% railes

%%
% Making small sections by x direction.
vxRail = vx.parent_cloud(railIdx,:);
vxRail = vxRail - mean(vxRail);
vxRail = vxRail * PcaFlattering(vxRail);

% figure; pcshow(vxRail, 'g');

maxX = max(vxRail(:,1));
minX = min(vxRail(:,1));

rail = false(size(vxRail,1),1); % rail
railY = false(size(vxRail,1),1); % rail only considering Y

sectionPreIds = [];
for i = minX:step:maxX
    
    minSec = i;
    maxSec =  minSec + step;
    
    %% Section
    sectionIds = find(vxRail(:,1) >= minSec & vxRail(:,1) < maxSec);
%     hold on; pcshow(vxRail(sectionIds,:), 'y');
    
    if length(sectionIds)/step < 100
        continue;
    end
    % Cloud of this section oriented
    vxSec = vxRail(sectionIds,:);
    vxSec = vxSec * PcaFlattering(vxSec);
    vxSec = vxSec - mean(vxSec);
    
    if range(vxSec(:,2)) > 2*railModel.dimensions(2) && length(sectionPreIds)/step > 100 % orienting this section using the previous section if this has a lot of dispersion in Y
        vxSecPre = vxRail(sectionPreIds,:);

        vxSec = vxRail(sectionIds,:);
        vxSec = vxSec * PcaFlattering(vxSecPre);
        vxSec = vxSec - mean(vxSec);
    end
%     figure; pcshow(vxSec, 'b', 'MarkerSize', 100);
    
    %% filtering by Y
%     figure;histogram(vxSec(:,2),'BinWidth',0.02);
    [N,edges]  = histcounts(vxSec(:,2),'BinWidth',0.02);
    [~,filter] = max(N);
    filter     = edges(filter);
    
    isRailY = vxSec(:,2) > (filter - railModel.dimensions(2)/2) & vxSec(:,2) < (filter + railModel.dimensions(2)/2);
    
%     hold on; pcshow(vxSec(isRailY,:), 'g', 'MarkerSize', 100);
    
    %% Filtering by Z
%     figure;histogram(vxSec(isRailY,3),'BinWidth',0.05);
%     isRailZ = vxSec(:,3) >= max(vxSec(isRailY,3)) - railModel.dimensions(3); 
    isRailZ = vxSec(:,3) >= mean(vxSec(isRailY,3));

    isRail = isRailY & isRailZ;
    
%     hold on; pcshow(vxSec(isRail,:), 'r', 'MarkerSize', 100);
    
    %% Saving idx

    rail(sectionIds(isRail)) = true;
    railY(sectionIds(isRailY)) = true;
    
    sectionPreIds = sectionIds(isRail);
    
%     hold on; pcshow(vxRail(sectionIds(isRail),:), 'r');
    
end

% figure; pcshow(vxRail, 'g');
% hold on; pcshow(vxRail(rail,:), 'r');

rail = railIdx(rail);
railY = railIdx(railY);

% figure; pcshow(vx.parent_cloud(railIdx,:), 'g');
% hold on; pcshow(vx.parent_cloud(railY,:), 'y');
% hold on; pcshow(vx.parent_cloud(rail,:), 'r');

end
