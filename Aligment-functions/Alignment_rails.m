clear all;% close all; clc
addpath Ferrocarril/Clouds_0519_cut_sec_and_divided/ Classes Matlab_help MyFunctions
plot_settings

%% Do you want saving?
want_save           =   true;
%% Define input and output folders
path_in     =   'Ferrocarril/Clouds_0519_cut_sec_and_divided/';
path_out    =   'Ferrocarril/Alignment_csv_files/';

% Load and las files
files = dir(['Ferrocarril/Clouds_0519_cut_sec_and_divided/','*_components.mat']);
TOT = length(files);
%%
prev_last_id    =   0;     
all_data        =   [];
error_clouds    =   0;
f   =   0;      
for file = files(11)' 
    f   =   f + 1;    
    file_name   =   file.name(1:end-15);
    bar_id      =   find(file_name == '_' );
    file_num    =   file_name(bar_id(1)+1:end);
    if prev_last_id == 0
        file_num_0  =   file_num;
    end
    disp([num2str(f), ' out of ', num2str(TOT), ': ', file_name])
    tic
    load([path_in,file_name,'.mat'])
%     load([path_in,file_name,'_sections.mat'])
    load([path_in,file_name,'_COMPONENTS.mat'])    

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Load rail coordinates and the draw line points
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rail_1  =   cut_cloud.Location(COMPONENTS.cloud_rails{1},:);
    rail_2  =   cut_cloud.Location(COMPONENTS.cloud_rails{2},:);
    
    L1      =   cat(2, COMPONENTS.rough_rails.line_1_points{:})';
    L1      =   L1(1:2:end,:); % note that the points are repeated. I may change this in the heuristic segmentation

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % For each line point, determine the closest point in the opposite rail
    % and then move this point to the centre of the rail in the same
    % direction.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [R_1_pts, R_2_pts, mid_pts]  =  compute_offset_pts2(rail_1, rail_2, L1);
    % Check if the overlap between clouds is good. If the connecting mid 
    % points are too close, neglect one of them.
    if f ~= 1
        % Check distance between connecting cloud points
        last_mid_pt     =   mid_pts(end,:);
        mid_pts         =   (R_2_pts + R_1_pts) / 2;    
        first_mid_pt    =   mid_pts(1,:);
        dist_mid_pts  	=	vecnorm((first_mid_pt - last_mid_pt), 2,2);
        if dist_mid_pts < 0.5
            R_1_pts  =   R_1_pts(2:end,:);
            R_2_pts  =   R_2_pts(2:end,:);
            mid_pts      =   (R_2_pts + R_1_pts) / 2;   
        end
    end
   
    vector_1        =   R_1_pts - mid_pts;
    vector_2        =   R_2_pts - mid_pts;
    offset_XY_1     =   vecnorm(vector_1(:,1:2),2,2) * -1;
    offset_XY_2     =   vecnorm(vector_2(:,1:2),2,2);
    offset_Z_1      =   vector_1(:,3);
    offset_Z_2      =   vector_2(:,3);
    offsets_id      =   (prev_last_id+1):length(mid_pts)+prev_last_id;
    prev_last_id    =   offsets_id(end);
    file_end_ids(f) =   prev_last_id;
    
    if any(offset_XY_1 < -1) || any(offset_XY_1 > -0.7) % Check point
        error_clouds = [error_clouds; file_num];
    end
    
    % Assign to all the points in the cloud the initial time for a later
    % filtering
    time    =   ones(length(mid_pts),1) * min(cut_cloud.timeStamp);
    % Store all values in a matrix
    data        =   [mid_pts, offset_XY_1, offset_XY_2, ...
                    offset_Z_1, offset_Z_2, R_1_pts, R_2_pts, time];    
    all_data    =   [all_data; data];

end

% %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Sort data by time and check for errors
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

all_data    =   sortrows(all_data, size(data,2));
% Compute director vector for assuring ascending direction. This is
% done by checking the angle between adjacent segments
all_rail_1_pts  =   all_data(:,8:10);
dxy_L1  =   all_rail_1_pts(2:end,:) - all_rail_1_pts(1:end-1,:);
dxy_L1  =   dxy_L1 ./ vecnorm(dxy_L1,2,2);
v1      =   dxy_L1(1:end-1,:);  
v2      =   dxy_L1(2:end,:);  
ang_bw_vec_1    =   acosd(dot(v1, v2,2));     
swapped_ids     =   find(abs(ang_bw_vec_1) > 50);
if ~isempty(swapped_ids)
    for a = 1:2:length(swapped_ids) 
        a_id  	=   swapped_ids(a);
        temp 	=   all_data(a_id,:);
        all_data(a_id,:)    =   all_data(a_id-1,:);
        all_data(a_id-1,:)  =   temp;            
    end
end

figure; plot3(all_data(:,1), all_data(:,2), all_data(:,3),'ro-')
hold on; plot3(all_data(:,8), all_data(:,9), all_data(:,10), 'bo-')
hold on; plot3(all_data(:,11), all_data(:,12), all_data(:,13), 'go-')
        
figure; pcshow(all_data(:,1:3), 'w', 'markersize', 120)
hold on; pcshow(all_data(:,8:10), 'g', 'markersize', 120)
hold on; pcshow(all_data(:,11:13), 'r', 'markersize', 120)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reorganise the data to store the offsets following ascendent offset id.
% Finally, write out the tables and export into a csv.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
if want_save
    % Store principal alignment data
    x   =	all_data(:,1);
    y   =   all_data(:,2);
    z   =   all_data(:,3);
    % Reorganise offset infotmation to have the indices as 1 1 2 2 3 3 ...
    offsets_id  =   (1:length(all_data))';
    offset_data =   [all_data(:,4), all_data(:,6), offsets_id; ...
                     all_data(:,5), all_data(:,7), offsets_id];
    offset_data =   sortrows(offset_data, 3);
    offsetXY    =   offset_data(:,1);
    offsetZ     =   offset_data(:,2);
    offset_id   =   offset_data(:,3);
    % Create tables
    T_offsets   =   table(offsetXY, offsetZ, offset_id);
    T_points    =   table(x, y, z);
    % Export tables
    writetable(T_offsets, [path_out,'PC_2_IFC_Rail_',file_num_0,'-', ...
        file_num,'_Offsets_Attempt1_02_09', '.csv'])
    
    writetable(T_points, [path_out,'PC_2_IFC_Rail_',file_num_0,'-', ...
        file_num,'_Main_Attempt1_02_09', '.csv'])
end
































