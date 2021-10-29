function [sec_ids, sec_traj_ids] = create_sections_no_rot(cloud, traj, spacing, width)
% Function that returns the sections of a point cloud by limiting the
% perpendicular distance to the trajectory and sectioning by number of 
% trajectory points.
%Inputs: 
% - cloud: Original pointCloud_ class object
% - traj: Object of trajectory class
% - spacing: Number of traj points to consider in each section
% - width: Desired width of the output cloud
%Outputs: 
%  - cut_cloud: cut cloud as a pointCloud_ class object
%  - sections_ids: sections of the cloud as a cell array
%  - sections_traj_ids: sections of the trajectory as a cell array
 
traj_pts    =   traj.points(:,1:2);
cloud_pts   =   cloud.Location(:,1:2);
cloud_pts   =   [(1:numel(cloud_pts)/2)',cloud_pts]; 

C_pts_rem   =   cloud_pts;
rem_idx       =   true(1,length(cloud_pts)); 
j = 0;
% figure; pcshow(cloud.Location, 'w')
% hold on; pcshow(ground_traj.points,'r','markersize', 80) 
    for i = 1:spacing:traj.count-spacing + 1
        j = j + 1;
%         j
%         if j == 30
%             a=0;
%         end
        % If the spacing is 1 modify code
        if i == traj.count-spacing + 1
            area_C_ids  =   C_pts_rem(:,1);
            area_cloud_pts  =   C_pts_rem(:,2:3);
        else
            % Define start and end trajectory points of the section as well 
            % as their direction
            pt_0        =   traj_pts(i,:);
            pt_0_p1     =   traj_pts(i+1,:);
            dxy_0       =   (pt_0_p1 - pt_0) / norm(pt_0_p1 - pt_0);

            if i == traj.count-spacing + 1
                pt_f        =   traj_pts(end,:);
                pt_f_m1     =   traj_pts(end - 1,:);        
            else
                pt_f        =   traj_pts(i + spacing,:);
                pt_f_m1     =   traj_pts(i + spacing - 1,:);
            end
            dxy_f       =   (pt_f - pt_f_m1) / norm(pt_f - pt_f_m1);

            % Create perpendicular lines to the initial and final points
            perp_f     =    [-dxy_f(2),dxy_f(1)];
            m_f        =    perp_f(2) / perp_f(1);
            perp_0     =    [-dxy_0(2),dxy_0(1)];
            m_0        =    perp_0(2) / perp_0(1);

            % Check the direction & Choose points whithin the range (as I  
            % am deleting the previous section no need to set lower bound)
            if  pt_0(:,2) - m_f * pt_0(:,1) < pt_f(2) - m_f * pt_f(1)   
                area_traj_ids   = 	traj_pts(:,2) - m_f * traj_pts(:,1) ...
                                    < pt_f(2) - m_f * pt_f(1) & ...
                                    traj_pts(:,2) - m_0 * traj_pts(:,1) ... 
                                    >= pt_0(2) - m_0 * pt_0(1);
                area_C_ids      = 	C_pts_rem(:,3)- m_f * C_pts_rem(:,2)...
                                    < pt_f(2) - m_f * pt_f(1);
            else
                area_traj_ids   = 	traj_pts(:,2) - m_f * traj_pts(:,1)...
                                    > pt_f(2) - m_f * pt_f(1) & ...
                                    traj_pts(:,2) - m_0 * traj_pts(:,1)...
                                    <= pt_0(2) - m_0 * pt_0(1);
                area_C_ids      = 	C_pts_rem(:,3)- m_f * C_pts_rem(:,2)...
                                    > pt_f(2) - m_f * pt_f(1);
            end

            area_traj_ids   =   find(area_traj_ids);
            area_cloud_pts  =   C_pts_rem(area_C_ids,2:3);

            area_C_ids      =   C_pts_rem(area_C_ids,1);

            if isempty(area_C_ids)
                j = j - 1;
                continue
            end
        end
        % Delete classified points from cloud
        rem_idx(area_C_ids)     =   false;    
        C_pts_rem               =   cloud_pts(rem_idx,:);
        
%         if i == traj.count-spacing + 1 && sum(remaining_idx) > 0
%             area_cloud_ids  =   [area_cloud_ids; cloud_pts_rem(:,1)];
%             area_cloud_pts  =   [area_cloud_pts; cloud_pts_rem(:,2:3)];
%             remaining_idx(area_cloud_ids)   =   false;
%         end

        % Cut the section by filtering the perpendicular distance to the 
        % trajectory computed as point to line distance
        D           =   point_to_line_distance(area_cloud_pts, pt_0, pt_f);
        area_C_ids  =   area_C_ids(D < width / 2);

        % Store indices
        sec_traj_ids{j}    =    area_traj_ids';
        sec_ids{j}         =    area_C_ids';

%     color = rand(1,3);
%     hold on; pcshow(cloud.Location(area_cloud_ids,:), color, 'markersize', 20)
%     hold on; pcshow(traj.points(area_traj_ids,:),color , 'markersize', 200)
    end
                  
end