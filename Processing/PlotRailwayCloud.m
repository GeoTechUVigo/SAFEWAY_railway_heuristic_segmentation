function [] = PlotRailwayCloud(cloud, components, fileName)
% Function to print the cloud with the components extracted, each one with
% its color.
%
% -------------------------------------------------------------------------
% INPUTS:
%
% cloud : pointCloud_
%
% components : cell array with the indexes of the componets extracted.
%
% fileName : char. Name of the cloud.
%                
% -------------------------------------------------------------------------
% OUTPUTS:
%                          
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 28/12/2020

% figure; pcshow(cloud.Location,[0.5,0.5,0.5]);

hold on; pcshow(cloud.Location,cloud.intensity);
% hold on; pcshow(cloud.Location(components.evaluated,:),'b');
% hold on; pcshow(cloud.Location(components.track,:),[0.5,0.5,1]);
title(fileName)
    if ~isempty(components.masts)
        hold on; pcshow(cloud.Location(cat(1,components.masts{:}),:), [0.6,0.6,0], 'MarkerSize', 50);
    end
    if ~isempty(components.bracketTunnel)
        hold on; pcshow(cloud.Location(cat(1,components.bracketTunnel{:}),:), [0.6,0.6,0], 'MarkerSize', 50);
    end
    if ~isempty(components.cables.others)
        hold on; pcshow(cloud.Location(cat(1,components.cables.others{:}),:), [1,0.5,1], 'MarkerSize', 50);
    end
    if ~isempty(components.cables.pairs)
        value = 0;
        color = [1,0];
        for j = 1:numel(components.cables.pairs)
            hold on; pcshow(cloud.Location(components.cables.pairs{j}{2},:), [color,0], 'MarkerSize', 50);
            hold on; pcshow(cloud.Location(components.cables.pairs{j}{1},:), [color/2,1], 'MarkerSize', 50);
            hold on; pcshow(cloud.Location(cat(1,components.rails{j}{:}),:), [color,0], 'MarkerSize', 50);

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
        hold on; pcshow(cloud.Location(cat(1,aux{:}),:), [0 0 0], 'MarkerSize', 50);
    end
    if ~isempty(components.signals.big)
        hold on; pcshow(cloud.Location(cat(1,components.signals.big{:}),:), 'y', 'MarkerSize', 50);
    end
    if ~isempty(components.signals.trafficLight)
        hold on; pcshow(cloud.Location(cat(1,components.signals.trafficLight{:}),:), [1,0.5,0.5], 'MarkerSize', 50);
    end
    if ~isempty(components.signals.light)
        hold on; pcshow(cloud.Location(cat(1,components.signals.light{:}),:), [0.7,0.7,0.9], 'MarkerSize', 50);
    end
    if ~isempty(components.signals.stone)
        hold on; pcshow(cloud.Location(cat(1,components.signals.stone{:}),:), [0.7,0.3,0.9], 'MarkerSize', 50);
    end
    if ~isempty(components.signals.inMast)
        hold on; pcshow(cloud.Location(cat(1,components.signals.inMast{:}),:), 'y', 'MarkerSize', 50);
    end
    
    WhitePcshow;
end

