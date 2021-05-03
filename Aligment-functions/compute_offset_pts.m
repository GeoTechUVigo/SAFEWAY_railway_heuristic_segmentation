function [R_1_pts, R_2_pts, mid_pts]  =   compute_offset_pts(R1, R2, L1)

    % Find the closest points in rail 2 to the line points of rail 1. These
    % will be points in the ground 
    perp_pt_2   =   R2(knnsearch(R2(:,1:2), L1(:,1:2)),:);

    % Select an area around the perpendicular points in rail 2 and around
    % the line points in rail 1
    areas_1     =   rangesearch(R1, L1, 0.4);
    areas_2     =   rangesearch(R2, perp_pt_2, 0.4);  
    
    % Identify the empty areas of rail 1 and remove the areas and line 
    % points if this is the case (this can be in regions where no points
    % are scanned but the algorithm keeps defining line points)
    empty_areas_1   =   cellfun(@(x)isempty(x), areas_1);
    
    areas_1     =   areas_1(~empty_areas_1);
    areas_2     =   areas_2(~empty_areas_1);
    
    L1	=   L1(~empty_areas_1,:);    
    
%
   
    [secs_L1,secs_rails] = divide_by_rails_curvature(L1,[R1;R2], 0, 2.1);
    secs_R1   =   cellfun(@(x) ismember(x, R1, 'rows'), ...
                    secs_rails, 'UniformOutput', false);
    secs_R2   =   cellfun(@(x) ismember(x, R2, 'rows'), ...
                    secs_rails, 'UniformOutput', false);
                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   NEW APPROACH (BELOW & COMMENTED, THE PREVIOUS APPROACH DOCUMENTED)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Create 11 points in each of the sections i.e., divide the rails
    % into 10 equally spaced sections.
    R_1_pts     =   [];
    R_2_pts     =   [];
    for i = 1:length(secs_rails)
        % Align and centre the clouds
        rail_sec    =   secs_rails{i} * pca(secs_rails{i});
        mins        =   min(rail_sec);
        maxs        =   max(rail_sec);
        
        avgs        =   mean([mins; maxs]);
        rail_sec    =   rail_sec - avgs;
        
        line_sec    =   L1(secs_L1{i},:) * pca(secs_rails{i});
        line_sec    =   line_sec - avgs;
        
        % Create 10 points in each section        
        r1_sec      =   rail_sec(secs_R1{i},:);
        r2_sec      =   rail_sec(secs_R2{i},:);
        
        
        pt_x_loc_1    =   min(r1_sec(:,1));
        pt_x_loc_2    =   min(r2_sec(:,1));
        
        if abs(pt_x_loc_1) < abs(pt_x_loc_2)
            obj_pt      =   r1_sec(r1_sec(:,1)==pt_x_loc_1,:);
            x_length    =   range(r1_sec(:,1));
            sec_length  =   x_length / 10;
        else
            obj_pt      =   r2_sec(r2_sec(:,1)==pt_x_loc_2,:);
            x_length    =   range(r2_sec(:,1));
            sec_length  =   x_length / 10;
        end
        
        R_1_ids     =   zeros(11, 1);
        R_2_ids     =   zeros(11, 1);
%         figure; pcshow(rail_sec)
        for j = 1:10+1
            % Define start point
            obj_line_ids    =   knnsearch(line_sec, obj_pt, 'K', 2);
            obj_line_ids    =   sort(obj_line_ids);
            obj_line_pts    =   line_sec(obj_line_ids,:);
            normal      	=   (obj_line_pts(2,:) - obj_line_pts(1,:));
            
%             % Create a plane for visualisation
%             PLANE           =   createPlane(obj_pt, normal);
%             drawPlane3d(PLANE);
%             hold on; pcshow(obj_pt, 'r', 'markersize', 400)

            % Select points around the plane            
            D_p         =   point_to_plane_distance(rail_sec,...
                            normal, obj_pt);
            plane_pts	=   rail_sec(D_p<0.015,:);
%             figure;plot(rail_sec(D_p<0.03,2),rail_sec(D_p<0.03,3),'ro')

            % Divide into left and right and select the top inner point for
            % each rail
            R1_P_pts    =   plane_pts(plane_pts(:,2) > 0,:);
            R2_P_pts    =   plane_pts(plane_pts(:,2) < 0,:);
            % Select the top in each rail
            top_ids     =   0.5 *(max(R1_P_pts(:,3)) + mean(R1_P_pts(:,3)));
            top_R1      =   R1_P_pts(R1_P_pts(:,3) > top_ids,:);
            
            top_ids     =   0.5 *(max(R2_P_pts(:,3)) + mean(R2_P_pts(:,3)));
            top_R2      =   R2_P_pts(R2_P_pts(:,3) > top_ids,:);
            
            % The top surface is that with the most common z (it is horz)
            [C, E]      =   histcounts(top_R1(:,3));
            % select top most freq
            select_C    =   find(C > 1, 1, 'last');
            surf        =   top_R1(top_R1(:,3) >= E(select_C) & top_R1(:,3) <= E(select_C+1),:);
            pt1          =   surf(surf(:,2) == min(surf(:,2)),:);
            
            [C, E]      =   histcounts(top_R2(:,3));
            select_C    =   find(C > 1, 1, 'last');
            surf        =   top_R2(top_R2(:,3) >= E(select_C) & top_R2(:,3) <= E(select_C+1),:);
            pt2         =   surf(surf(:,2) == max(surf(:,2)),:); 
            
            if ~isempty(pt2) && ~isempty(pt1)
                R_2_ids(j)  =   find(rail_sec == pt2,1);
                R_1_ids(j)  =   find(rail_sec == pt1,1);
            end
            
            
            % Reset objective point
            obj_pt      =   obj_pt + sec_length * normal/norm(normal);
            obj_pt      =   rail_sec(knnsearch(rail_sec, obj_pt, 'K', 1),:);
        end
        R_1_ids     =   R_1_ids(R_1_ids ~= 0);
        R_2_ids     =   R_2_ids(R_2_ids ~= 0);
        R_1_pts     =   [R_1_pts; secs_rails{i}(R_1_ids,:)];
        R_2_pts     =   [R_2_pts; secs_rails{i}(R_2_ids,:)];
        
%             figure; pcshow(secs_rails{i})
%             hold on; pcshow(R_1_pts, 'W', 'markersize', 400)
%             hold on; pcshow(R_2_pts, 'W', 'markersize', 400)
    end
    
    % Compute the principal alignment point as the centre of the rails
    mid_pts         =   (R_2_pts + R_1_pts) / 2;
    
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
% 
%     % Increase the height of the points to select the top inner rail point
%     high_L_11   =   [L1(:,1:2), L1(:,3) + 0.5];
%     normal      =   zeros(length(L1), 3);
%     R_2_pts     =   zeros(length(L1), 3);
%     for i = 1:length(L1)
%         v1      =   L1(i,:);
%         v2      =   perp_pt_2(i,:);
%         v3      =   high_L_11(i,:);
%         v4      =   [perp_pt_2(i,1:2), perp_pt_2(i,3) + 0.2];
%         
%         normal(i,:)  =   cross((v3 - v1), (v2 - v1));
%         
%         pts     =   R2(areas_2{i},:);
%         D1      =   point_to_plane_distance(pts, normal(i,:), v1);
%         width   =   0.01; 
%         perp_pts    =   pts(D1 <=  min(D1) + width,:);
%         
%         while length(perp_pts) < 20
%             width = width + 0.005;
%             perp_pts    =   pts(D1 <=  min(D1) + width,:);
%         end
%         
%         D1      =   D1(D1 <=  min(D1) + width,:);
%         
%                 
%         D2   =   pdist2(perp_pts,  v4);
%         act_min     =    min(D2);
%         prev_min    =   min(D2);
%         while min(D2) > 0.05 && act_min < prev_min%v4(3) > perp_pt_2(i,3) + 0.14 
%             prev_min    =   min(D2);
%             v4(3)   =   v4(3) - 0.02; 
%             D2      =   pdist2(perp_pts,  v4);
%             act_min     =    min(D2);
%         end
%             
%         
%         final_pt   = find(D2 == min(D2));
%         if length(final_pt) > 1
%             H           =   perp_pts(final_pt,3);
%             final_pt    =   final_pt(H == max(H));
%             if length(final_pt) > 1
%                 final_pt    =   final_pt(D1(final_pt) == min(D1(final_pt)));
%                 if length(final_pt) > 1
%                     final_pt    =   final_pt(D1(final_pt) == min(D1(final_pt)));
%                     if length(final_pt) > 1
% %                         figure; pcshow(pts, 'markersize', 10)
% %                         hold on; pcshow(perp_pts, 'w', 'markersize', 50)
% %                         hold on; pcshow(perp_pts(final_pt,:),'g', 'markersize', 400)
%                         final_pt    =   final_pt(1);
%                     end
%                 end
%             end
%         end
%         
%         R_2_pts(i,:)    =   perp_pts(final_pt,:);
%         
%         
%         
%         
% %         figure; pcshow(pts, 'markersize', 10)
% %         hold on; pcshow(perp_pts, 'w', 'markersize', 50)
% %         hold on; pcshow(v2, 'c', 'markersize', 350)
% %         hold on; pcshow(v4, 'c', 'markersize', 350)
% %         hold on; pcshow(R_2_pts(i,:), 'r', 'markersize', 350)
% %         close
%     end
%        
%     
%     R_1_pts     =   zeros(length(L1), 3);
%     
%     for i = 1:length(L1)
%         v1      =   R_2_pts(i,:);  
%         
%         pts     =   R1(areas_1{i},:);
%         D1      =   point_to_plane_distance(pts, normal(i,:), v1);
%         perp_pts    =   pts(D1 <=  min(D1) +0.01,:);
%         D1          =   D1(D1 <=  min(D1) +0.01);
%         
%         perp_D      =   pdist2(perp_pts, perp_pt_2(i,:));
%         min_perp    =   find(perp_D == min(perp_D));
%         if length(min_perp) > 1
%             H       = perp_pts(min_perp,3);
%             min_perp = min_perp(H == min(H));
%             if length(min_perp) > 1
%                 min_perp    =   min_perp(D1(min_perp) == min(D1(min_perp)));
%             end
%             if length(min_perp) > 1
%                 min_perp    =   min_perp(1); % tanto ten
%             end
%         end
%         
%         perp_pt_1   =   perp_pts(min_perp,:);
%         v4          =   [perp_pt_1(1:2), perp_pt_1(3) + 0.2];
%         
%         D2      =   pdist2(perp_pts,  v4);
%         final_pt   = find(D2 == min(D2));
%         if length(final_pt) > 1
%             H           =   perp_pts(final_pt,3);
%             final_pt    =   final_pt(H == max(H));
%             if length(final_pt) > 1
%                 final_pt    =   final_pt(D1(final_pt) == min(D1(final_pt)));
%                 if length(final_pt) > 1
% %                     figure; pcshow(pts, 'markersize', 10)
% %                     hold on; pcshow(perp_pts, 'w', 'markersize', 50)
% %                     hold on; pcshow(perp_pts(final_pt,:),'g', 'markersize', 400)
%                     final_pt    =   final_pt(1);
%                 end
%             end
%         end
%         R_1_pts(i,:)    =   perp_pts(final_pt,:);
%     end
%     
% %     R_2_pts         =   areas_2(knnsearch(areas_2, high_L_11),:);    
% %     higher_line_2   =   [R_2_pts(:,1:2), R_2_pts(:,3) + 2];    
% %     R_1_pts         =   areas_1(knnsearch(areas_1, higher_line_2),:);
%     
%     % Compute the principal alignment point as the centre of the rails
%     mid_pts         =   (R_2_pts + R_1_pts) / 2;
%     
%     figure; pcshow([R1; R2])
%     hold on; pcshow([R_1_pts; R_2_pts], 'w', 'markersize', 400)
%     close
%     
end