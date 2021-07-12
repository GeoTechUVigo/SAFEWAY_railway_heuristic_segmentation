%% Main code to extract elements in a railway
% It is need a trajectory in .txt and a cloud in .las
% Each .las is transformed into a pointCloud_
%
%
% In the variable model there are the datas of the element that are going
% to be extracted.
%
% The pointCloud_ and the trajecroty are splitted in sections, discarding
% points far away to the trajectory.
%
% The cloud is voxelized. Not all the cloud, just points that are in any
% section.
%
% The voxelized cloud is evaluated looking for elements. This evaluation 
% is done section by section, but the final result is referred to the .las
%
% The results are saved modifying the cloud .las. If pathOut is not
% specificated, the input cloud is overwrited.
%
% -------------------------------------------------------------------------
% INPUTS:
% 
% pathInTrajectory: char. Path of the trajectory.            
%
% pathIn: char. Path with all the .las.
%
% pathOut : char. Path to save compontens indexes and status.
%
% pathErrors : char. Path to save errors in the code.
%
% Data of the elements specified in GenerateElements().
%
% -------------------------------------------------------------------------
% OUTPUTS:
%
% cloud : .las. The cloud is saved segmented.

% 
% status : cell array. Execution times.
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 28/12/2020

%% Cleaning 
clear all; close all; clc;

%% Using the righ symbol for path (windows, linux, mac)
if(isunix)                                                                       
    symb='/';                   
else
    symb='\'; 
end 

%% Adding folders and subfolders
addpath Classes Preprocessing Processing

%% Paths
pathInTrajectory = 'D:\Trabajo\Clouds\Chamartín-Ávila Ferrocarril\Trajectories';
pathInCloud      = 'D:\Trabajo\Clouds\Chamartín-Ávila Ferrocarril\Originales';
pathOut          = 'D:\Trabajo\Clouds\Chamartín-Ávila Ferrocarril\Procesados_13_05_21';
pathOutStatus    = 'D:\Trabajo\Clouds\Chamartín-Ávila Ferrocarril\Status_13_05_21';

%% Reading files .las and traj
traj = TrajectoryGenerate(pathInTrajectory, 'orthometricToElipsoidal', true); % trajectory of all the clouds
list = dir(strcat(pathInCloud, symb, '*.las')); % list with all the clouds

%% Data to extraction
% Elements' model
pathModels = [pwd, '\Models'];

model = GenerateElements([pathModels,symb]);

%% Analyzing all the clouds
grid = 0.1;
parfor i = 1:numel(list)
    try
        status = [];
        status.total = tic;

        % Generation pointCloud_
        status.lasToPointCloud = tic;
        cloud = las2pointCloud_(strcat(pathInCloud, symb, list(i).name)); % Create pointCloud_
        status.lasToPointCloud = toc(status.lasToPointCloud);

        % Sectioning the cloud
        status.sectioning = tic;
        [cloud, trajCloud, sections, idxCloud, idxTraj] = Sectioning(cloud,traj);
        status.sectioning = toc(status.sectioning);

        % Voxelazing
        status.voxelize = tic;
        cloud = Voxels(cloud,grid);
        status.voxelize = toc(status.voxelize);

        % Segmentation
        status.segmentation  = tic;
        [components, status] = Segmentation(cloud, trajCloud, sections, idxCloud, model, status);
        status.segmentation = toc(status.segmentation);
        
        status.total = toc(status.total);

        % Saving
        status.save = tic;
        cloud = []; % cloud .las is read in ModifySaveLas, so cloud is delete to free up memory. clear cloud is not applicable in parfor loop
        ModifySaveLas(strcat(pathInCloud, symb, list(i).name),components, 'pathOut', strcat(pathOut, symb, list(i).name)); 
        status.save = toc(status.save);
            
    catch ME
        status = getReport(ME);
    end
    
    SaveParallel(strcat(pathOutStatus, symb, erase(list(i).name, '.las'), '_status.mat'), status);

end