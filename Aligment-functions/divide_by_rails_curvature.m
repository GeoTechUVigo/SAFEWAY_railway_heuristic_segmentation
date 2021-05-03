function [section_cloud,section_rails] = divide_by_rails_curvature(varargin)%cloud_loc, rails, overlap, width_lim)
% Function that creates sections of a railway point cloud with reference to
% its curvature. This is checked using the width of the rails when these are
% aligned. If the section is straight, the total width of the rails will be
% less than 2.5 m as the standard separation is 1.668 m.
%   Inputs: 
%   - cloud: point cloud data as a pointCloud_ object
%   - rails: location of the rails in the cloud as a 3D array
%   Outputs: 
%   - section_cloud: cell array containing the indices of the cloud that
%                   correspond to each section
%   - section_rails: cell array containing the coordinates of the rails
%                   that lay within each section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




if length(varargin) == 2
    cloud_loc   =   varargin{1};
    rails       =   varargin{2};
    overlap     =   2; 
    width_lim   =   2.5;
elseif length(varargin) == 4
    cloud_loc   =   varargin{1};
    rails       =   varargin{2};
    overlap     =   varargin{3};
    width_lim   =   varargin{4};
end


if ~isa(cloud_loc, 'double')
    cloud_loc = cloud_loc.Location;
end

original_ids    =   1:length(rails);
overlap_points  =   [0, 0, 0];
cut             =   true;
rails_loc       =   rails;
i   =   0;

while cut
    cut = false;
    i = i + 1;    
    rails_center    =   rails_loc - mean(rails_loc);
    rails_pca       =   rails_center * pca(rails_center);
    % Determine the width pf the aligned rails. If this is larger than 
    % about 2 m this means that the section is curved  
    rails_width     =   max(rails_pca(:,2)) - min(rails_pca(:,2));
    while rails_width >= width_lim
        cut             =   true;
        division_ids    =   (rails_pca(:,1) < overlap);
        overlap_ids     =   (rails_pca(:,1) < overlap & rails_pca(:,1) > 0);
        overlap_ids     =   original_ids(overlap_ids);
        overlap_points  =   rails_loc(overlap_ids,:);
        original_ids    =   original_ids(division_ids);
        cut_rails       =   rails_pca(division_ids, :); 
        rails_center    =   cut_rails - mean(cut_rails);
        rails_pca       =   rails_center * pca(rails_center);
        rails_width     =   max(rails_pca(:,2)) - min(rails_pca(:,2));
    end
    
    section_rails{i}    =   rails_loc(original_ids,:);
    stored_rails_pts    =   cat(1, section_rails{:});
    overlap_ids         =   find(ismember(rails, overlap_points, 'rows'));
    remaining_ids       =   find(~ismember(rails, stored_rails_pts, 'rows'));
    remaining_ids       =   cat(1,remaining_ids, overlap_ids);
    rails_loc           =   rails(remaining_ids,:);
    original_ids        =   1:length(rails_loc);
    % Store section of the  cloud
    rot_cloud   =   (cloud_loc - mean(section_rails{i})) * pca(section_rails{i});
    rot_rails   =   (section_rails{i} - mean(section_rails{i})) * pca(section_rails{i});
    xmax        =    max(rot_rails(:,1));
    xmin        =    min(rot_rails(:,1));
    if cut
        section_cloud{i}    =   find(rot_cloud(:,1) <= xmax & rot_cloud(:,1) > xmin)'; 
    else
        section_cloud{i}    =   find(rot_cloud(:,1) > xmin)';
    end
    
%     color   = rand(1,3);
%     hold on; pcshow(section_rails{i},color);
%     hold on; pcshow(cloud_loc(section_cloud{i},:), color);
end

