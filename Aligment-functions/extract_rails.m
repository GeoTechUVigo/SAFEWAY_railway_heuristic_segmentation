function [RR] = extract_rails(SEC, cloud, traj, start_pt)
% Function that creates a line along the rail top points and identifies the 
% real rail points by extracting the whole cloud points that are within a
% threshold sitance to this line
% Inputs:
%  - SEC   - struct containing for a group of sections: (1) sections (all 
%            the points); (2) track_id (tack points of the section); (3)
%            traj_interval (two trajectory points corresponding to the 
%            section); (4) traj_yaw and (5) traj_pitch (trajectory yaw and
%            pitch identified for each section);  and (6) RR, a second
%            structure containgin the indices of a rough rail extraction
%            and the tips of this rails in each section
%  - cloud - pointCloud_ object to which the sections refer
%  - traj  - trajectory object to which the sections refer
% Output:
%  - RR - same struct as in the input SEC.RR with the addition of the 
%         indices of whole rails extracted

% figure
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Loop going forwards
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Rough rails 
    RR  =   SEC.RR;
    draw_pt_3  =   RR.tips_1{start_pt}';
    draw_pt_4  =   RR.tips_2{start_pt}';
    for i = start_pt:numel(SEC.sections)
%             i
%     if i == 30
%         a = 0;
%     end
        flag_1  =   false;      flag_2  =   false;
        % Compute trajectory unit vector        
        traj_pt_1   =   traj.points(SEC.traj_interval(i-1,1),:);
        traj_pt_2   =   traj.points(SEC.traj_interval(i-1,2),:);
        dxyz        =   (traj_pt_2 - traj_pt_1)';  
        dxy     =   [dxyz(1:2,:); 0];  
        dxy     =   dxy / norm(dxy); 
        dxyz    =   dxyz / norm(dxyz);         
        % Define head points of the segment using the previous tail point. 
        % Note that the start_point is always defined so that the previous 
        % point has only one tip in each rail
        if i == start_pt
            draw_pt_1    =   RR.tips_1{i-1}'; 
            draw_pt_2    =   RR.tips_2{i-1}';
        else
            draw_pt_1    =   draw_pt_3;
            draw_pt_2    =   draw_pt_4;
        end  
        
        % Define section points
        sec_ids         =   SEC.sections{i};
        sec_pts         =   cloud.Location(sec_ids,3) < ...
                            max([traj_pt_1(3); traj_pt_2(3)]) + 1;
        low_pts         =   sec_ids(sec_pts);
        ground_sec      =   select(cloud,low_pts);
        section_pts     =   ground_sec.Location;
        
        % Define final draw points
        if i == numel(SEC.sections)   % Last section of the cloud            
            draw_pt_3  =   extend_pt_i(draw_pt_1, dxyz, SEC.sec_width, ground_sec);
            draw_pt_4  =   extend_pt_i(draw_pt_2, dxyz, SEC.sec_width, ground_sec);
        else % Case with more rails or objects
            if RR.nTips(i,1) ~= 0  && ~isempty(RR.tips_1{i})        
                % Find the next point on the rail using the pair of points
                % with the closest direction to that of the trajectory
                dxy_1       =   RR.tips_1{i}' - draw_pt_1;
                dxy_1       =   dxy_1 / norm(dxy_1);
                dxy_1(3,:)  =   0;
                
                cross_p     =   cross(repmat(dxy,1,size(dxy_1,2)),dxy_1,1);
                dot_p       =   dot(repmat(dxy,1,size(dxy_1,2)),dxy_1,1);
                ang_bw_vecs        =   atan2d(vecnorm(cross_p),dot_p);
                [min_d3, draw_pt_3_i]   =   min(ang_bw_vecs);
                
                if min_d3 > 0.9
                    flag_1  =   true;
                else
                    % FIX HEIGHT ISSUE
                    P3_dxyz     =   draw_pt_1 + dxyz * SEC.sec_width / 2;
                    if abs(P3_dxyz(3) - RR.tips_1{i}(draw_pt_3_i,3)) > 0.075
                        flag_1      =   true;
                    else
                        draw_pt_3   =   RR.tips_1{i}(draw_pt_3_i,:)';
                    end
                end
            else % If there are no tips, keep the previous segment
                flag_1  =   true;
            end
            % Similarly, for rail 2
            if RR.nTips(i,2)  ~= 0 && ~isempty(RR.tips_2{i})
                % Find the next point on the rail using the pair of points
                % with the closest direction to that of the trajectory
                dxy_2       =   RR.tips_2{i}' - draw_pt_2;
                dxy_2       =   dxy_2 / norm(dxy_2);   
                dxy_2(3,:)  =   0;      
                
                % Find the angle between the current trajectory direction
                % and the direction of the segment in the current section
                cross_p     =   cross(repmat(dxy,1,size(dxy_2,2)),dxy_2,1);
                dot_p       =   dot(repmat(dxy,1,size(dxy_2,2)),dxy_2,1);
                ang_bw_vecs =   atan2d(vecnorm(cross_p),dot_p);
                [min_d4, draw_pt_4_i]   =   min(ang_bw_vecs); 
                
                if min_d4 > 0.9
                    flag_2  =   true;
                else
                    % FIX HEIGHT ISSUE
                    P4_dxyz     =   draw_pt_2 + dxyz * SEC.sec_width / 2;
                    if abs(P4_dxyz(3) - RR.tips_2{i}(draw_pt_4_i,3)) > 0.075
                        flag_2      =   true;
                    else
                        draw_pt_4   =   RR.tips_2{i}(draw_pt_4_i,:)';
                    end
                end  
            else
                flag_2      =   true;
            end
            % When there are no tips or the segment is too short, use the 
            % trajectory direction to draw the line. Move the point to fit
            % within the segment points
            if flag_1  
                draw_pt_3  =   extend_pt_i(draw_pt_1, dxyz, SEC.sec_width, ground_sec);
            end
            if flag_2
                draw_pt_4  =   extend_pt_i(draw_pt_2, dxyz, SEC.sec_width, ground_sec);
            end
        end     

        % The tips might be too close for drawing a line with a significant
        % direction. Check this and store the segment tips 
        segment_1_length = norm((draw_pt_1 - draw_pt_3),2);
        segment_2_length = norm((draw_pt_2 - draw_pt_4),2);
        if segment_1_length < 1
            draw_pt_3  =   extend_pt_i(draw_pt_1, dxyz, SEC.sec_width, ground_sec);
        end
        if segment_2_length < 1
            draw_pt_4  =   extend_pt_i(draw_pt_2, dxyz, SEC.sec_width, ground_sec);
        end
               
        % Centre the drawpoint in the rail
        draw_pt_3   =   check_centre(draw_pt_1, draw_pt_3, dxyz, ground_sec);
        draw_pt_4   =   check_centre(draw_pt_2, draw_pt_4, dxyz, ground_sec);
        
                

        % Store start and ending points of the segment
        segment_pts_1   =   [draw_pt_1, draw_pt_3];
        segment_pts_2   =   [draw_pt_2, draw_pt_4];
        % Find the points within a 8 cm distance in the XY plane
      
        [d_1, ~]    =   point_to_line_distance(section_pts(:,1:2), ...
                        segment_pts_1(1:2,1), segment_pts_1(1:2,2));
        [d_2, ~]    =   point_to_line_distance(section_pts(:,1:2), ...
                        segment_pts_2(1:2,1), segment_pts_2(1:2,2));
        % Store segment points and select and stotr the final rail points 
        if i ~= start_pt
            RR.line_1_points{i}  =   segment_pts_1; 
            RR.line_2_points{i}  =   segment_pts_2; 
        else
            RR.line_1_points{i}  =   draw_pt_3; 
            RR.line_2_points{i}  =   draw_pt_4; 
        end
        RR.rail_1_id{i}  =  low_pts(d_1 < 0.08 & ...
                            section_pts(:,3)>min(segment_pts_1(3,:))-0.5& ...
                            section_pts(:,3)<max(segment_pts_1(3,:))+ 0.1);
        RR.rail_2_id{i}  =  low_pts(d_2 < 0.08 & ...
                            section_pts(:,3)>min(segment_pts_1(3,:))-0.5& ...
                            section_pts(:,3)<max(segment_pts_2(3,:))+ 0.1);    
% 

%        figure
%     hold on; pcshow(cloud.Location(cat(2,RR.rail_1_id{:}),:),'w')
%     hold on; pcshow(cloud.Location(cat(2,RR.rail_2_id{:}),:),'y')
%     hold on; pcshow(cat(2,RR.line_1_points{:})','r', 'markersize', 400)
%     hold on; pcshow(cat(2,RR.line_2_points{:})','c', 'markersize', 400)

% if i > 25
%     hold on; pcshow(ground_sec.Location)
%     hold on; pcshow([draw_pt_1, draw_pt_2]','b','markersize', 400)
%     hold on; pcshow([draw_pt_3, draw_pt_4]','r','markersize', 400)
%     hold on; pcshow(cloud.Location(RR.rail_1_id{i},:),'w')
%     hold on; pcshow(cloud.Location(RR.rail_2_id{i},:),'c')
%     hold on;
% end
%     if flag_2 
%     hold on; pcshow(RR.tips_2{i}, 'g', 'markersize', 400)
%     end
% %     close

%         L1 = createLine3d(draw_pt_3', draw_pt_1');
%         drawLine3d(L1, 'r')
%         L1 = createLine3d(draw_pt_4', draw_pt_2');
%         drawLine3d(L1, 'g')
    
    end
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Loop going backwards
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = start_pt-1:-1:1
        flag_1  =   false;      flag_2  =   false;        
        % Define the first point of the segment
        if i == start_pt-1
            draw_pt_1   =  RR.line_1_points{i+1};   
            draw_pt_2   =  RR.line_2_points{i+1};
        else            % Use the previous end point as first point
            draw_pt_1   =   draw_pt_3;
            draw_pt_2   =   draw_pt_4;                                 
        end
    
        % Define final points
        if i == 1         % End of the cloud
            % Define section points        
            sec_ids         =   SEC.sections{i};
            sec_pts         =   cloud.Location(sec_ids,3) < ...
                                max([traj_pt_1(3); traj_pt_2(3)]) + 1;
            low_pts         =   sec_ids(sec_pts);
            ground_sec      =   select(cloud,low_pts);
            section_pts     =   ground_sec.Location;
            % Define points by extending the previous
            draw_pt_3   =   extend_pt_i(draw_pt_1, dxyz, SEC.sec_width, ground_sec);
            draw_pt_4   =   extend_pt_i(draw_pt_2, dxyz, SEC.sec_width, ground_sec);
        else
            % Compute trajectory unit vector
            traj_pt_1   =   traj.points(SEC.traj_interval(i-1,1),:);
            traj_pt_2   =   traj.points(SEC.traj_interval(i-1,2),:);
            dxyz        =   -(traj_pt_2 - traj_pt_1)'; 
            dxy     =   [dxyz(1:2,:); 0];  
            dxy     =   dxy / norm(dxy); 
            dxyz    =   dxyz / norm(dxyz); 
            % Define section points        
            sec_ids         =   SEC.sections{i};
            sec_pts         =   cloud.Location(sec_ids,3) < ...
                                max([traj_pt_1(3); traj_pt_2(3)]) + 1;
            low_pts         =   sec_ids(sec_pts);
            ground_sec      =   select(cloud,low_pts);
            section_pts     =   ground_sec.Location;
            if RR.nTips(i,1) ~= 0 && ~isempty(RR.tips_1{i})% Case with more rails or objects
                % Find the next point on the rail
                dxy_1       =   RR.tips_1{i}' - draw_pt_1;
                dxy_1(3,:)  =   0;
                dxy_1       =   dxy_1 / norm(dxy_1);   
                
                % Use the pair of points with the closest direction to the
                % trajectory. Compute angel between trajectory and the
                % draw points direction   
                cross_p     =   cross(repmat(dxy,1,size(dxy_1,2)),dxy_1,1);
                dot_p       =   dot(repmat(dxy,1,size(dxy_1,2)),dxy_1,1);
                ang_bw_vecs =   atan2d(vecnorm(cross_p),dot_p);
                [min_d3, draw_pt_3_i]   =   min(ang_bw_vecs);

                if min_d3 > 0.9
                    flag_1  =   true;
                else
                    % FIX HEIGHT ISSUE
                    P3_dxyz     =   draw_pt_1 + dxyz * SEC.sec_width / 2;
                    if abs(P3_dxyz(3) - RR.tips_1{i}(draw_pt_3_i,3)) > 0.075
                        flag_1      =   true;
                    else
                        draw_pt_3   =   RR.tips_1{i}(draw_pt_3_i,:)';
                    end
                end
            else
                flag_1  =   true;
            end
            % Similarly, for rail 2       
            if RR.nTips(i,2) ~= 0 && ~isempty(RR.tips_2{i})
                % Find the next point on the rail
                dxy_2       =   RR.tips_2{i}' - draw_pt_2;
                dxy_2(3,:)  =   0;   
                dxy_2       =   dxy_2 / norm(dxy_2);    
                
                % Use the pair of points with the closest direction to the
                % trajectory. Compute angel between trajectory and the
                % draw points direction            
                cross_p     =   cross(repmat(dxy,1,size(dxy_2,2)),dxy_2,1);
                dot_p       =   dot(repmat(dxy,1,size(dxy_2,2)),dxy_2,1);
                ang_bw_vecs  =  atan2d(vecnorm(cross_p),dot_p);
                [min_d4, draw_pt_4_i]   =   min(ang_bw_vecs);
                
                if min_d4 > 0.9
                    flag_2  =   true;
                else
                    % FIX HEIGHT ISSUE
                    P4_dxyz     =   draw_pt_2 + dxyz * SEC.sec_width / 2;
                    if abs(P4_dxyz(3) - RR.tips_2{i}(draw_pt_4_i,3)) > 0.075
                        draw_pt_4   =   P4_dxyz;
                    else
                        draw_pt_4   =   RR.tips_2{i}(draw_pt_4_i,:)';
                    end
                end   
            else
                flag_2  =   true;
            end                
            % When there are no tips or the segment is too short, use the 
            % trajectory direction to draw the line
            if flag_1
                draw_pt_3   =   extend_pt_i(draw_pt_1,dxyz,SEC.sec_width,ground_sec);
            end
            if flag_2
                draw_pt_4   =   extend_pt_i(draw_pt_2,dxyz,SEC.sec_width,ground_sec);
            end
        end

        % The tips might be too close for drawing a line with a significant
        % direction. Check this and store the segment tips
        segment_1_length    =   norm((draw_pt_1 - draw_pt_3),2);
        segment_2_length    =   norm((draw_pt_2 - draw_pt_4),2);
        if segment_1_length < 1
            draw_pt_3   =   extend_pt_i(draw_pt_1, dxyz, SEC.sec_width, ground_sec);
        end
        if segment_2_length < 1
            draw_pt_4   =   extend_pt_i(draw_pt_2, dxyz, SEC.sec_width, ground_sec);
        end
        
        % Redefine draw points to fit to the centre of the top of the rail 
        draw_pt_3       =   check_centre(draw_pt_1, draw_pt_3, dxyz, ground_sec);
        draw_pt_4       =   check_centre(draw_pt_2, draw_pt_4, dxyz, ground_sec);
        
        % Store start and ending points of the segment
        segment_pts_1   =   [draw_pt_3, draw_pt_1];
        segment_pts_2   =   [draw_pt_4, draw_pt_2];
        
        
        % Find the points within a 10 cm radius from the drawn line
        [d_1, ~]    =   point_to_line_distance(section_pts(:,1:2), ...
                        segment_pts_1(1:2,1), segment_pts_1(1:2,2));
        [d_2, ~]    =   point_to_line_distance(section_pts(:,1:2), ...
                        segment_pts_2(1:2,1), segment_pts_2(1:2,2));                             

        % Store start and ending points of the segment
        RR.line_1_points{i}  =   segment_pts_1; 
        RR.line_2_points{i}  =   segment_pts_2; 
                    
                    
        % Extract and store final rail points
        RR.rail_1_id{i}  =  low_pts(d_1 < 0.08 & ...
                            section_pts(:,3)>min(segment_pts_1(3,:))-0.5 & ...
                            section_pts(:,3)<max(segment_pts_1(3,:))+0.1);
        RR.rail_2_id{i}  =  low_pts(d_2 < 0.08& ...
                            section_pts(:,3)>min(segment_pts_2(3,:))- 0.5& ...
                            section_pts(:,3)<max(segment_pts_2(3,:))+ 0.1);    
    
%     hold on; pcshow(cloud.Location(RR.rail_1_id{i},:),'w')
%     hold on; pcshow(cloud.Location(RR.rail_2_id{i},:),'c')
%     hold on; pcshow(segment_pts_1','r','markersize', 400)
%     hold on; pcshow(segment_pts_2','r','markersize', 400) 
    
    end
  
  
end % function

function draw_pt_f  =   extend_pt_i(draw_pt_i, dxyz, width, ground_sec)

    draw_pt_f       =   draw_pt_i + dxyz * width;

    ground_pts  =   ground_sec.Location;
    D_R         =   point_to_line_distance(ground_pts, draw_pt_i, draw_pt_f);
    
    % First selection to define a high point nearby
    neighbours  =   ground_pts(D_R<0.2,:);
    n_d         =   pdist2(neighbours, draw_pt_f');
    neighbours  =   neighbours(n_d < 0.2,:);
    upper_pts   =   (neighbours(:,3) - draw_pt_f(3)) > 0.08;
    if sum(upper_pts) > 20
        draw_pt_f  	=  	mean(neighbours(upper_pts,:));
        D_R         =   point_to_line_distance(ground_pts, draw_pt_i, draw_pt_f);
    end
    
    top_pts    =   ground_pts(D_R<0.02,:);
    
    if isempty(top_pts)
        r_min       =   min(pdist2(ground_sec.Location, draw_pt_f'));
        if r_min > 0.5
            return
        end
        top_pts     =   ground_pts(D_R<r_min,:);
    end
    
    % Move the point to the centre of the section
    close_pt    =   knnsearch(top_pts, draw_pt_i');
    close_pt    =   top_pts(close_pt,:)';
    vec         =   (close_pt - draw_pt_i)/norm(close_pt - draw_pt_i);
    close_pt    =   close_pt + vec * width / 2;
    close_pt    =   knnsearch(top_pts, close_pt');
    if ~isempty(close_pt)
        draw_pt_f   =   top_pts(close_pt,:)';
    end

end

function draw_pt_f  =   check_centre(draw_pt_i, draw_pt_f, dxyz, ground_sec)
        ground_pts  =   ground_sec.Location;
        
        
%         vec_X         =   (draw_pt_f - draw_pt_i)/norm(draw_pt_f - draw_pt_i);
        vec_X         =     dxyz;
        vec_Y         =   [vec_X(2), - vec_X(1), 0];
        vec_Z         =   cross(vec_X, vec_Y);
        
        % Choose points close to the rail to ease the computation later
        D_R         =   point_to_line_distance(ground_pts(:,1:2), ...
                            draw_pt_f(1:2), draw_pt_i(1:2));
                        
        ground_pts  =   ground_pts(D_R < 0.10, :);
        D_R         =   D_R(D_R < 0.10, :);
        
        if length(ground_pts)<100
            % There are no points around the draw point - Some clouds
            % finish with an empty triangle
%             if isempty(ground_pts)
                return 
%             elseif rangesearch(
        end
        
         %Choose the points corresponding to the top of the rail
        [D_V, ~, S_V]   =   point_to_plane_distance(ground_pts, vec_Z',...
                            draw_pt_f');
                        
        % Define the top points to be positive and select them
        H_p1        =   ground_pts(find(S_V == 1,1), 3);
        H_m1        =   ground_pts(find(S_V == -1,1), 3);
        
        if H_p1 > H_m1
            D_V         =   D_V .* S_V;
        else
            D_V         =   D_V .* -S_V;
        end
        
        % Select points of the top. If the original point is already very
        % hugh this might lead to errors. Thus, check the amount of points
        % avobe it
        top_pts     =   ground_pts(D_V >- 0.015 & D_V < 0.2,:);
        low_margin  =   -0.015;
        actual_width    =   max(pdist(ground_pts(D_V < 0.2,1:2)));
        top_width       =   max(pdist(top_pts(:,1:2)));
        if isempty(top_width); top_width = 0; end % in case there is only 1 top_point
        
        while top_width <= min(actual_width * 0.9, 2.7) || length(top_pts) < 100 
            % figure; pcshow(ground_pts); hold on; pcshow(top_pts, 'w', 'markersize', 20)
            low_margin  =   low_margin - 0.02; 
            top_pts     =   ground_pts(D_V > low_margin & D_V < 0.2,:);
            top_width   =   max(pdist(top_pts(:,1:2)));
            if isempty(top_width); top_width = 0; end 
        end
        D_R         =   D_R(D_V >low_margin & D_V < 0.2);
        
        if isempty(top_pts)
            figure; pcshow(ground_sec.Location)
            hold on; pcshow(draw_pt_f', 'r', 'markersize', 800)
            return
        end
        % Determine the distances of the cloud points to the final draw
        % point
        perp_pt     =   draw_pt_f + vec_Y';
        [D_P, S_P]  =   point_to_line_distance(top_pts(:,1:2), ...
                            draw_pt_f(1:2), perp_pt(1:2));
        D_P         =   D_P .* S_P;
        
        % First, check longitudinal distance. Move point towards the centre
        % of the section
        restrict_top_ids  =   (D_R < 0.01);
        restrict_top_pts  =   top_pts(restrict_top_ids,:);
        restrict_D_P        =   D_P(restrict_top_ids);
        r   =   0.01;
        while length(restrict_top_pts) < 80 
            r   =   r + 0.01;
            restrict_top_ids  =   (D_R < r);
            restrict_top_pts  =   top_pts(restrict_top_ids,:);
            restrict_D_P        =   D_P(restrict_top_ids);  
        end
        
        if min(restrict_D_P) > - 1.3
            addition        =   min(restrict_D_P) + 1.5;
            [~,centre_id]   =   min(abs(restrict_D_P - addition));
            draw_pt_f       =   restrict_top_pts(centre_id,:)';
        elseif max(restrict_D_P) < 1.3 && actual_width > 2.5
            addition        =   max(restrict_D_P) - 1.5;
            [~,centre_id]   =   min(abs(restrict_D_P - addition));
            draw_pt_f       =   restrict_top_pts(centre_id,:)';
        end
        
        % Second, check the transversal distance. Move the point to the 
        % middle of the rail width (approx 7.5 cm)

        X       =   draw_pt_f(1);
        Y       =   draw_pt_f(2);
        Z       =   draw_pt_f(3);
        X_g     =   ground_pts(:,1);
        Y_g     =   ground_pts(:,2);
        Z_g     =   ground_pts(:,3);
        
        subcloud_pts    =   ground_pts((X_g - X).^2 + (Y_g - Y).^2 <= ...
                            0.1^2 & abs(Z_g - Z) < 0.02,:);        
        z_lim           =   0.02;
        while length(subcloud_pts) < 20 && z_lim < 0.25
            z_lim = z_lim + 0.01;
            subcloud_pts    =   ground_pts((X_g - X).^2 + (Y_g - Y).^2 <= ...
                    0.1^2 & abs(Z_g - Z) < z_lim,:);  
        end
        [D_T, S_T]      =   point_to_line_distance(subcloud_pts(:,1:2), ...
                            draw_pt_i(1:2), draw_pt_f(1:2));
        D_T             =   D_T .* S_T ;
        
        % Remove isolated points or separated rails
        if std(subcloud_pts(:,3))> 0.01 || (max(D_T)-min(D_T)) > 0.15
            whole   =   one_region_growing(subcloud_pts, 0.035, draw_pt_f'); 
            if sum(whole == 1) < 10
                whole   =   one_region_growing(subcloud_pts, 0.05, draw_pt_f'); 
            end
            subcloud_pts    =   subcloud_pts(whole == 1,:);
            [D_T, S_T]      =   point_to_line_distance(subcloud_pts(:,1:2), ...
                                draw_pt_i(1:2), draw_pt_f(1:2));
            D_T             =   D_T .* S_T ;
        end
        
        
        if (max(D_T)-min(D_T)) > 0.15 % the beggining of a rail crossing. Move towards the shortest length
            P_dxyz = extend_pt_i(draw_pt_i, dxyz, 3, ground_sec);
            [~, s] = point_to_line_distance(P_dxyz(1:2)', ...
                            draw_pt_i(1:2), draw_pt_f(1:2));
            addition = range(D_T(S_T == s)) / 2 * s;
            [~, i]      =   min(abs(D_T-addition));
            draw_pt_f   =   subcloud_pts(i, :)';
        end
        
        if (sum(S_T==1)/sum(S_T~=0) < 0.3)
%                 figure;         
%                 hold on; pcshow(draw_pt_f', 'r', 'markersize', 800)
%                 hold on; pcshow(subcloud_pts(S_T==1,:), 'r', 'markersize', 150)
%                 hold on; pcshow(subcloud_pts(S_T==-1,:), 'g', 'markersize', 150)
%                 L = createLine3d(draw_pt_i',draw_pt_f');
%                 drawLine3d(L, 'y')

            if min(D_T(D_T > 0)) > 0.01
                % Remove points belonging to rails nearby
                subcloud_pts    =   subcloud_pts(D_T < 0.01, :);
                D_T             =   D_T(D_T < 0.01);
            end
            
            if max(D_T) < 0.03
                addition    =   max(D_T) - 0.04;
                [~, i]      =   min(abs(D_T-addition));
                draw_pt_f   =   subcloud_pts(i, :)';
            end
                
%                 hold on; pcshow(draw_pt_f', 'b', 'markersize', 800)
        elseif sum(S_T==-1)/sum(S_T~=0) < 0.3
            if max(D_T(D_T < 0)) < -0.01
                % Remove points belonging to rails nearby
                subcloud_pts    =   subcloud_pts(D_T > -0.01, :);
                D_T             =   D_T(D_T > -0.01);
            end
            
            if min(D_T) > - 0.03
                addition    =   min(D_T) + 0.04;
                [~, i]      =   min(abs(D_T-addition));
                draw_pt_f   =   subcloud_pts(i, :)';
                
            end
        end

end
   


                