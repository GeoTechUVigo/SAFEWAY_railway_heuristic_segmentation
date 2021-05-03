function [cloud] = CloudGeneration(varargin)
% Function to read .las and generate a pointCloud_. They can be merged in
% one pointCloud_.
% 
% -------------------------------------------------------------------------
% INPUTS:
%
% pathIn: char. Path with the files with the .las 
% 
% wantSave: logical.
% 
% pathOut : char. Path with the file to save. If it is empty pathOut = pathInt
% 
% merge: logical. Merging .las in one cloud
%
% -------------------------------------------------------------------------
% OUTPUTS:
%
% cloud : pointCloud_.
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 22/12/2020

% Checking inputs
parser = inputParser;
parser.addRequired('pathIn', @(x)validateattributes(x,{'char'}, {'nonempty'})); 
parser.addParameter('wantSave', false,@(x)validateattributes(x,{'logical'}, {'nonsparse'}));
parser.addParameter('pathOut', string([]),@(x)validateattributes(x,{'char'}, {'nonsparse'}));
parser.addParameter('merge', false,@(x)validateattributes(x,{'logical'}, {'nonsparse'}));
parser.parse(varargin{:});

% Inputs
pathIn   = parser.Results.pathIn;
wantSave = parser.Results.wantSave;
pathOut  = parser.Results.pathOut;
merge    = parser.Results.merge;
% Reading the file
if(isunix)                                                                       
    symb='/';                   
else
    symb='\'; 
end 
list = dir(strcat(pathIn, symb, '*.las'));

% Merge all the .las in one cloud
if merge
    for i = 1:numel(list)
    % Generate pointCloud_
    cloud     = las2pointCloud_(strcat(pathIn, symb, list(i).name));
    clouds{i} = cloud;
    end

    cloud = MergePointCloud_(clouds);
    
    % Save
    if wantSave

        if isempty(pathOut)
            pathOut = pathIn;
        end

        cloudName = list(1).name;
        cloudName =  erase(cloudName, '.las');
        save(strcat(pathOut, symb, cloudName), 'cloud');

    end
% Generate a cloud for each .las
else
    for i = 1:numel(list)
        
        cloud = las2pointCloud_(strcat(pathIn, symb, list(i).name));
        
        
        % Save
        if wantSave

            if isempty(pathOut)
                pathOut = pathIn;
            end

            cloudName = list(i).name;
            cloudName =  erase(cloudName, '.las');
            save(strcat(pathOut, symb, cloudName), 'cloud');

        end
    end
end

