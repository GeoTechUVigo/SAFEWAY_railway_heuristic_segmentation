function [traj] = TrajectoryGenerate(varargin)
% Generating trajectory object from .txt file in pathIn
% 
% -------------------------------------------------------------------------
% INPUTS:
%
% pathIn: char. Path with the file of the trajecroty .txt
% 
% wantSave: logical
% 
% pathOut: char. where Trajectory will be saved
% 
% orthometricToElipsoidal: logical. Transform the 3 coordinate from 
%                          orthonometric to elipsoidal. Default = false
%                
% -------------------------------------------------------------------------
% OUTPUTS:
%
% traj : Trajectory
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 28/12/2020

% Checking inputs
parser = inputParser;
parser.addRequired('pathIn', @(x)validateattributes(x,{'char'}, {'nonempty'})); 
parser.addParameter('wantSave', false,@(x)validateattributes(x,{'logical'}, {'nonsparse'}));
parser.addParameter('pathOut', string([]),@(x)validateattributes(x,{'char'}, {'nonsparse'}));
parser.addParameter('orthometricToElipsoidal', false,@(x)validateattributes(x,{'logical'}, {'nonsparse'}));
parser.parse(varargin{:});


if(isunix)                                                                       
    symb='/';                   
else
    symb='\'; 
end 
    
% Inputs
pathIn = parser.Results.pathIn;
wantSave = parser.Results.wantSave;
pathOut = parser.Results.pathOut;
orthometricToElipsoidal = parser.Results.orthometricToElipsoidal;

% Reading the file
list = dir(strcat(pathIn, symb, '*.txt'));
fileID = fopen(strcat(pathIn, symb,list(1).name));
C = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 'HeaderLines', 25, 'Delimiter', '\t');
fclose(fileID);

% If it is need calculate elipsoial height
if orthometricToElipsoidal
    C{5} = C{5} - geoidheight(C{6}, C{7});
end

% Generate trayectory
traj = trajectory([C{3}, C{4}, C{5}], C{1}, C{9}, C{10}, C{11});

% Save
if wantSave
    
    if isempty(pathOut)
        pathOut = pathIn;
    end
    
    save(strcat(pathOut, symb, 'trajectory'), 'traj');
    
end

end

