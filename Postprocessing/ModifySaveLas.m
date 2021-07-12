function [] = ModifySaveLas(varargin)
% Function to modify and save a .las including the results of the
% segmentation process.
%
% If pathOut is not specified, the cloud is overwrited.
% It is saved with the same name as the input .las
% 
% The following atributes are modified as follows:
%
% cloud.record.classification --> specifies the type of element.
%                                 rail -----------> 1
%                                 catenary -------> 2
%                                 contact --------> 3
%                                 dropper --------> 4
%                                 other wires ----> 5
%                                 mast -----------> 6
%                                 sign -----------> 7
%                                 traffic light --> 8
%                                 mark -----------> 9
%                                 sign in mast ---> 10
%                                 light ----------> 11
%
% cloud.record.user_data --> specifies the index of the track. The elements
%                            of a track are rails, contact, catenary and 
%                            droppers.
%                            The index of the rails of each track has a 
%                            power of ten. Each contact-catenary-dropper 
%                            group corresponding to the same track, has a
%                            unique index between the power of ten of this
%                            track and the next power. It only works if no
%                            track has more than 9 contact-catenary-dropper
%                            groups.                        
%
% cloud.record.point_source_id --> specifies the number of the element.
%                                  Each group has a unique index.
%
% -------------------------------------------------------------------------
% INPUTS:
%
% pathIn: char. Path of the .las 
%
% pathOut : char. Path to save the modifyed .las. If it is not specified
%                 the cloud the .las is overwrited. The .las is saved with
%                 the same name as the input .las
% 
% components : cell array of indexes. Indexes of each element segmented
%
% -------------------------------------------------------------------------
% OUTPUTS:
%
% cloud : .las. The cloud is saved segmented.
%
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 09/04/2021

%% Checking inputs
parser = inputParser;
parser.addRequired('pathIn', @(x)validateattributes(x,{'char', 'string'}, {'nonempty'})); 
parser.addRequired('components', @(x)validateattributes(x,{'struct'}, {'nonempty'})); 
parser.addParameter('pathOut', string([]),@(x)validateattributes(x,{'char', 'string'}, {'nonsparse'}));
parser.parse(varargin{:});
    
% Inputs
pathIn = parser.Results.pathIn;
components = parser.Results.components;
pathOut = parser.Results.pathOut;

if isempty(pathOut)
    pathIn = pathOut;
end

%% Loading cloud
cloud = LASread(char(pathIn));

%% Initialising
cloud.record.classification(:) = 0;
cloud.record.point_source_id(:) = 0;
cloud.record.user_data(:) = 0;
group = 1;

%% Masts
for i = 1:numel(components.masts)
    % Classification
    cloud.record.classification(components.masts{i})= 6; 
    % Groups. Save in pont_source_id
    cloud.record.point_source_id(components.masts{i})= group;
    group = group + 1;
end

%% Big signs
for i = 1:numel(components.signals.big)
    % Classification
    cloud.record.classification(components.signals.big{i})= 7;
    % Groups. Save in pont_source_id
    cloud.record.point_source_id(components.signals.big{i})= group;
    group = group + 1;
end

%% Traffic lights
for i = 1:numel(components.signals.trafficLight)
    % Classification
    cloud.record.classification(components.signals.trafficLight{i})= 8;
    % Groups. Save in pont_source_id
    cloud.record.point_source_id(components.signals.trafficLight{i})= group;
    group = group + 1;
end

%% Marks
for i = 1:numel(components.signals.stone)
    % Classification
    cloud.record.classification(components.signals.stone{i})= 9;
    % Groups. Save in pont_source_id
    cloud.record.point_source_id(components.signals.stone{i})= group;
    group = group + 1;
end

%% Lights
for i = 1:numel(components.signals.light)
    % Classification
    cloud.record.classification(components.signals.light{i})= 11;
    % Groups. Save in pont_source_id
    cloud.record.point_source_id(components.signals.light{i})= group;
    group = group + 1;
end

%% Signs in masts
for i = 1:numel(components.signals.inMast)
    % Classification
    cloud.record.classification(components.signals.inMast{i})= 10;
    % Groups. Save in pont_source_id
    cloud.record.point_source_id(components.signals.inMast{i})= group;
    group = group + 1;
end

%% Other cables
for i = 1:numel(components.cables.others)
    % Classification
    cloud.record.classification(components.cables.others{i})= 5;
    % Groups. Save in pont_source_id
    cloud.record.point_source_id(components.cables.others{i})= group;
    group = group + 1;
end

%% Tracks
for i = 1:numel(components.rails)
    % Classification
    cloud.record.classification(components.rails{i}{1})           = 1; % left rail
    cloud.record.classification(components.rails{i}{2})           = 1; % right rail
    for j = 1:numel(components.cables.pairs{i})
        cloud.record.classification(components.cables.pairs{i}{j}{1})    = 2; % catenary
        cloud.record.classification(components.cables.pairs{i}{j}{2})    = 3; % contact
        cloud.record.classification(cat(2,components.droppers{i}{j}{:})) = 4; % droppers
    end

    % Groups. Saving them in pont_source_id
    cloud.record.point_source_id(components.rails{i}{1}) = group;
    group = group + 1;
    cloud.record.point_source_id(components.rails{i}{2}) = group;
    group = group + 1;
    for j = 1:numel(components.cables.pairs{i})
        cloud.record.point_source_id(components.cables.pairs{i}{j}{1}) = group;
        group = group + 1;
        cloud.record.point_source_id(components.cables.pairs{i}{j}{2}) = group;
        group = group + 1;
        for k = 1:numel(components.droppers{i}{j})
            cloud.record.point_source_id(components.droppers{i}{j}{k}) = group;
            group = group + 1;
        end
    end

    % Structures
    cloud.record.user_data(components.rails{i}{1}) = i * 10;
    cloud.record.user_data(components.rails{i}{2}) = i * 10;
    for j = 1:numel(components.cables.pairs{i})
        cloud.record.user_data(components.cables.pairs{i}{j}{1}) = i*10+j;
        cloud.record.user_data(components.cables.pairs{i}{j}{2}) = i*10+j;
        cloud.record.user_data(cat(2,components.droppers{i}{j}{:})) = i*10+j;
    end
end

 LASwrite(cloud,char(pathOut));

end

% location = [cloud.record.x,cloud.record.y,cloud.record.z];
% figure; pcshow(location,'b');
% 
% i = 0;
% i = i+1;
% 
% color = rand(1,2);
% hold on; pcshow(location(cloud.record.point_source_id == i,:),[color,0], 'MarkerSize', 50);
% 
% i = 0;
% i = i+1;
% 
% color = rand(1,2);
% hold on; pcshow(location(cloud.record.user_data == i,:),[color,0], 'MarkerSize', 50);



