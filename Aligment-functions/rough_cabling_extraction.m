function [rough_cabling] = rough_cabling_extraction(nT, rails, masts, tensors, catinvers, tlight)
% Function that extracts a rough segmentation of the rails from the non-
% track points of a point cloud.
% Inputs:
% - nT: pointCloud_ object containing the non-track points. 
% - rails: 3D array containing the coordinates of the rails.
% -	masts: cell array containing the mast poles points.
% -	tensors: cell array containing the cabling tensors points.
% -	catinvers: cell array containing the mast cativers points.
% -	tlight: cell array containing the traffic light points.
% Outputs:
% -	rough_cabling:  indices of the cables rough extraction.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Align and centre the cloud and mast components to the rails
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rails_pca   =   pca(rails, 'Economy', false);
    rails_loc   =   rails * rails_pca;
    centre_pt  	=   mean(rails_loc);
    
    notT_loc    =   nT.Location * rails_pca - centre_pt;
    rails_loc  	=   rails_loc - centre_pt;

    masts_loc   =   cellfun(@(x) x * rails_pca - centre_pt, masts, ...
                    'UniformOutput', false);
    tensors_loc =   cellfun(@(x) x * rails_pca - centre_pt, tensors,...
                    'UniformOutput', false);
    catin_loc   =   cellfun(@(x) x * rails_pca - centre_pt, catinvers, ...
                    'UniformOutput', false);                
    tlight_loc   =   cellfun(@(x) x * rails_pca - centre_pt, tlight, ...
                    'UniformOutput', false);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Select upper points only and remove points far from the rails
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    top_masts       =   cellfun(@(x) max(x(:,3)), masts_loc);    
    high_ids        =   find(notT_loc(:,3) > min(top_masts) - 3 ...
                        & notT_loc(:,3) < max(top_masts) + 1);
    high_nT_loc     =   notT_loc(high_ids,:);
    VX_high         =   Voxels(high_nT_loc, 0.1);
    VX_rails        =   Voxels(rails_loc, 1);    
    [~,D]           =   knnsearch(VX_high.Location(:,1:2),  ...
                        VX_rails.Location(:,1:2));
                    
    if sum(D < 3) < 100; 	rough_cabling	=   [];   return;	end    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Perform a one region growing using the masts as seeds
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    masts_loc   =   cat(1, masts_loc{:}, tensors_loc{:},...
                    catin_loc{:}, tlight_loc{:}); 
    cabling     =   one_region_growing(VX_high.Location, 0.5, masts_loc);
    cabling     =   VX_high.parent_pt_idx(cabling == 1);
    cabling     =   high_ids(cat(1, cabling{:}));
    
                    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Remove masts
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rough_cabling_loc   =   notT_loc(cabling, :);
    rough_cabling_ids   =   ~ismember(rough_cabling_loc,masts_loc, 'rows');
    rough_cabling       =   cabling(rough_cabling_ids);

end






























