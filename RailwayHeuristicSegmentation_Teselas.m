%% Main code to extract elements in a railway
% It is need a trajectory in .csv and a cloud in tiles. Besides, it is need
% a matrix that shows which tiles are close to each trajectory point.
%
% This script needs LAStools.
%
% The trajectory is sectioned. For each section, the tiles of these
% trajectory points are loaded. These tiles make a unique cloud. This cloud
% is save in a temporary file becuase it is created using LAStools. 
%
% In the variable model there are the datas of the element that are going
% to be extracted, saving it in .laz.
%
% The cloud is loaded, the remote points are deleted and it is downsampled.
% the temporary file is deleted and this cloud is saved in .laz to free 
% memory because the process works with a voxelised cloud. This is the 
% cloud that is going to be segmented.
%
% The cloud is voxelised.
%
% The voxelised cloud is evaluated looking for the elements, but the final 
% result is referred to the .laz.
%
% The results are saved modifying the cloud .laz.
%
% The status of the process is saved in a .json file.
%
% -------------------------------------------------------------------------
% INPUTS:
% 
% pathInTrajectory: char. Path of the trajectory.            
%
% pathInCloud: char. Path with all the .laz.
%
% pathOut : char. Path to save compontens indexes and status.
%
% pathOutStatus : char. Path to save the status.
%
% pathCloudsOfTrajectories: char. Path of the matrix of correlation tiles -
% trajectory points.
%
% output_temp: char. Path to save temporary files.
%
% Data of the elements specified in GenerateElements().
% -------------------------------------------------------------------------
%
% OUTPUTS:
%
% cloud : cloud segmented in .laz formed with tiles of 300 m of railway.
% 
% status : cell array with teh status of the process.
% -------------------------------------------------------------------------
%
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 19/07/2021.
% -------------------------------------------------------------------------

%% Cleaning 
clear all; close all; clc;

%% Using the righ symbol for path (windows, linux, mac)
if(isunix)                                                                       
    symb='/';                   
else
    symb='\'; 
end 

%% Adding folders and subfolders
addpath Classes Preprocessing Processing Postprocessing

%%
grid = 0.1;

%% Paths
pathInTrajectory = "C:\Users\xeotecnoloxias\Desktop\Dani\dani_uk\clouds\trajectories";
pathInCloud      = "C:\Users\xeotecnoloxias\Desktop\Dani\dani_uk\clouds\reduced_laz_files";
pathOut          = "C:\Users\xeotecnoloxias\Desktop\Dani\dani_uk\clouds\output";
pathOutStatus    = "C:\Users\xeotecnoloxias\Desktop\Dani\dani_uk\clouds\status";
pathCloudsOfTrajectories = "C:\Users\xeotecnoloxias\Desktop\Dani\dani_uk\clouds\in_range";
output_temp = "C:\Users\xeotecnoloxias\Desktop\Dani\dani_uk\SAFEWAY_railway_heuristic_segmentation\archivos_temp";

%% Data to extraction
% Elements' model
pathModels = [pwd, '\Models'];

model = GenerateElements([pathModels,symb]);

%% Reading trajectory files
list_traj = dir(strcat(pathInTrajectory, symb, '*.csv')); % list with all the trajectories
list_relation = dir(strcat(pathCloudsOfTrajectories, symb, '*.csv')); % list with all the matrix clouds - trajectory points
list_clouds = dir(strcat(pathInCloud, symb, '*.laz')); % list with all the clouds

%% All the trajectories. It is created in this way to use a trajectory object correctly
traj = cell(numel(list_traj),1);
for i = 1:numel(list_traj)
    traj{i} = trajectory(0,0,0);
    points = readtable(strcat(pathInTrajectory, symb, list_traj(i).name));
    traj{i}.points = points{:,:};
    traj{i}.points(:,3) = traj{i}.points(:,3) + 1; % this traj is on the floor
    traj{i}.timeStamp = zeros(length(traj{i}.points),1);
end

%% Sectioning each trajectory, loading the clouds of each section, removing lateral points using all the trajectories and segmenting it.
% .laz that have been analysed in other sections are not analaysed now.
for i = 1:numel(list_traj)
    %% Clouds of this trajectory
    matrix_clouds = readtable(strcat(pathCloudsOfTrajectories, symb, list_relation(i).name));
    matrix_clouds = matrix_clouds{:,:};
    clouds_traj = false(size(matrix_clouds));
    clouds_traj(matrix_clouds == "True") = true;
    
    %% Create a matrix where to know which .laz have been analysed
    if i == 1
        analysed_laz = false(1,size(clouds_traj,2));
    end
    
    %% Removing analysed laz
    clouds_traj(:,analysed_laz) = false;
    
    %% Updating analysed_laz
    analysed_laz(any(clouds_traj,1)) = true;
    
    %% Sectioning trajectory
    sections_traj = Sectioning_trajectory(traj{i});
    
    %% Analyse the cloud of each trajectory section
    parfor j = 1:length(sections_traj)
        process = [];
        status = [];
        
        try 
            status.total = tic; 
            status.load = tic;
            
            process = "select";
            
            sections = [];   
            trajCloud = select(traj{i}, sections_traj{j});
            
            if isempty(trajCloud)
               error('no trajCloud points');
            end
            
            %% Clouds of this section
            process = "Clouds of this section";
            clouds_traj_sec = clouds_traj(sections_traj{j},:);
            list_clouds_traj = list_clouds(any(clouds_traj_sec,1));

            if isempty(list_clouds_traj)
               error('no list_clouds_traj'); 
            end
            
            %% Load clouds of this section
            process = "Load clouds of this section";
            % Merge all the points clouds using lastools, save it and load it
            input = string();
            for k = 1:length(list_clouds_traj)
                input = input + pathInCloud + symb + list_clouds_traj(k).name + " ";
            end
            file_temp = strcat(output_temp, symb,string(i), "_", string(j), ".las");
            process = "system call";
            system("lasmerge -i " + input + "-o "  + file_temp);

            process = "loading temp file";
            % Cloud with all the teselas merged
            cloud = las2pointCloud_(strcat(output_temp, symb,string(i), "_", string(j), ".las"));
            % Delete this cloud from disk
            process = "delete temp cloud";
            delete(file_temp);
            
            status.load = toc;
            
            %% Remove remote points
            status.cut_remote = tic;
            process = "Remove remote points";
            max_dist = 10;
            remote = remote_points(cloud.Location, trajCloud, traj, max_dist);
            cloud = select(cloud, find(~remote));
            if isempty(cloud.Location)
                error('all points are remote'); 
            end
            status.cut_remote = toc;
            
            %% Downsampling
            status.downsampling = tic;
            process = "Downsample";
            gridStep = 0.03;
            cloud = pcdownsample(pointCloud(cloud.Location, 'Intensity', double(cloud.intensity)),'gridAverage',gridStep);
            cloud = pointCloud_(cloud.Location, 'intensity', uint16(cloud.Intensity));
            status.downsampling = toc;
%             figure; pcshow(cloud.Location);

            %% Save this cloud to free RAM memory and work just with the voxelized one. This will be rewrited with the segmentation information.
            status.save_no_segmented = tic;
            process = "Save cloud no segmented";
            file_out = strcat(pathOut, symb, string(i), "_" + string(j), ".las");
            SaveLas(cloud, file_out);
            status.save_no_segmented = toc;
            
            %% Voxels
            process = "Voxels";
            status.voxelize = tic;
            cloud = Voxels(cloud, grid);
            status.voxelize = toc(status.voxelize);

            %% Segmentation
            process = "Segmentation";
            % apaño para reutilizar el codigo. No se modifica la nube por lo que no haria falta
            sections{1}.cloud = cat(1, cloud.parent_idx{:});
            sections{1}.traj = 1:length(trajCloud.points);
            idxCloud = 1:length(cloud.parent_cloud);

            status.segmentation  = tic;
            [components, status] = Segmentation(cloud, trajCloud, sections, idxCloud, model, status);
            status.segmentation = toc(status.segmentation);

            %% Saving
            process = "Save .las segmented";
            status.save = tic;
            cloud = []; % cloud .las is read in ModifySaveLas, so cloud is delete to free up memory. clear cloud is not applicable in parfor loop
            ModifySaveLas(file_out, components, 'pathOut', file_out);
            % Save as .laz
            process = "Save .laz segmented";
            system(strcat("las2las -i ", file_out, " -o ", erase(file_out, '.las'), ".laz"));
            delete(file_out);
            status.save = toc(status.save);

            status.total = toc(status.total);
            
        catch ME
            status = [];
            status.error = getReport(ME);
            status.process = process;
        end
          
        %% Save status
        fid = fopen(strcat(pathOutStatus, symb, string(i), "_", string(j), '_status.json'), 'w');
        fprintf(fid, jsonencode(status,'PrettyPrint',true));
        fclose(fid);
        
%         SaveParallel(strcat(pathOutStatus, symb, string(i), "_", string(j), '_status.mat'), status);
    end
end