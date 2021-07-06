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

%%
grid = 0.1;
gird_save = 0.01;

%% Paths
pathInTrajectory = "D:\Trabajo\Clouds\UK\trajectories";
pathInCloud      = "D:\Trabajo\Clouds\UK\reduced_laz_files";
pathOut          = "D:\Trabajo\Clouds\UK\reduced_laz_files\output";
pathOutStatus    = "D:\Trabajo\Clouds\UK\reduced_laz_files\status";
pathCloudsOfTrajectories = "D:\Trabajo\Clouds\UK\in_range";

%% Data to extraction
% Elements' model
pathModels = [pwd, '\Models'];

model = GenerateElements([pathModels,symb]);

%% Reading trajectory files
list_traj = dir(strcat(pathInTrajectory, symb, '*.csv')); % list with all the trajectories
list_clouds = dir(strcat(pathInCloud, symb, '*.laz')); % list with all the clouds

for i = 1:numel(list_traj)
    %% Trajectory. It is created in this way to use a trajectory object correctly
    traj = trajectory(0,0,0);
    points = readtable(strcat(pathInTrajectory, symb, list_traj(i).name));
    traj.points = points{:,:};
    traj.points(:,3) = traj.points(:,3) + 1; % this traj is on the floor
    traj.timeStamp = zeros(length(traj.points),1);
    
    %% Clouds of this trajectory
    matrix_clouds = readtable(strcat(pathCloudsOfTrajectories, symb, list_traj(i).name));
    matrix_clouds = matrix_clouds{:,:};
    clouds_traj = false(size(matrix_clouds));
    clouds_traj(matrix_clouds == "True") = true;
    
    %% Sectioning trajectory
    sections_traj = Sectioning_trajectory(traj);
    trajCloud = traj; % esto se hace así porque los de UK no mandaron trayectoria. Para no modificar el código se hace una con los XYZ nada más leidos de un csv
    %% Analyse the cloud of each trajectory section
    for j = 1:length(sections_traj)
        trajCloud.points = traj.points(sections_traj{j},:);
        
        %% Load clouds of this section
        clouds_traj_sec = clouds_traj(sections_traj{j},:);
        list_clouds_traj = list_clouds(any(clouds_traj_sec,1));
        if isempty(list_clouds_traj)
            break;
        end
        
        % Merge all the points clouds using lastools, save it and load it
        input = string();
        for k = 1:length(list_clouds_traj)
            input = input + pathInCloud + "\" + list_clouds_traj(k).name + " ";
        end
        
        output = "D:\Trabajo\SAFEWAY2020\Segmentacion\SAFEWAY_railway_heuristic_segmentation\Archivos\temp.las";
        system("lasmerge -i " +input + "-o " + output);
        
        cloud = las2pointCloud_(output);
        
        % Voxelazing
        status.voxelize = tic;
        cloud = Voxels(cloud,grid);
        status.voxelize = toc(status.voxelize);
        
        % apaño por adaptar el codigo. No se modifica la nube por lo que no haria falta
        sections{1}.cloud = cat(1, cloud.parent_idx{:});
        sections{1}.traj = 1:length(trajCloud.points);
        idxCloud = 1:length(cloud.parent_cloud);
        
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
        
    end
end