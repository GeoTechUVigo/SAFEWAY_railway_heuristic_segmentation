function [rough_points]= Rails_raster_detect(track, gs)
% Function that rasterises the ground cloud and performs a classification
% according to the average intensity and height range of each pixel. Then,
% the selected points are voxelised and clusterised using a region growing
% algorithm. 
% Inputs:
%  - ground: obect pointCloud_
%  - gs: grid size
% Output:
%  - rough_points: array of the selected track indices selected as rails.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Rasterise the ground using the provided grid size
    raster_ground = raster2DANM(track, gs);
    % Extract heights and intensity images
    I_h     =   raster_ground.height_image;
    I_i     =   raster_ground.intensity_image;
    % Binarise images
    I_bin_h  =  imbinarize(I_h,'adaptive','Sensitivity',0.4);
    I_bin_i  =  imbinarize(I_i,'adaptive','Sensitivity',0.6);
    % Combine both imputs into a composed image and separate the "white"  
    % pixels from the "black" ones. 
    I_comp      =   I_bin_h - I_bin_i;  
    indices     =   regionprops(I_comp,'PixelIdxList').PixelIdxList; 

    recover_ids     =   ismember(raster_ground.indices, indices);
    recover_points  =   raster_ground.parent_idx(recover_ids);
    recover_points  =   cell2mat(recover_points);
    rough_points    =   recover_points;

end
%     figure;imshow(I_comp)