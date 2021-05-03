function [SEC] = create_sections(cloud, traj, sec_width)
% Function that divides the input cloud into a group of sections with a
% length of y_distance.
% Inputs:
% - cloud: obect pointCloud_
% - traj: object trajectory
% - sec_width: length of the sections in metres
% Output:
% - SEC: struct containing 
%   · sections: cell array containing the cloud indices for each section 
%   · traj_sections: cell array containing the traj indices in each sec
%   · traj_yaw: array containing the yaw of closest traj point to each sec
%   · traj_pitch: array containing the pitch of closest traj point to each sec


    cloud_T     =   transpose(cloud.Location);
    whole_cloud_ids         =   1:cloud.Count;
    cloud_T_ids =   [whole_cloud_ids; cloud_T];
    traj_T      =   transpose(traj.points);
    % Find the first point for rotation as the closest point of the cloud
    % to the first trajectory point
    [~, initial_point]   =   min(vecnorm(cloud_T - traj_T(:,1)));
    initial_point       =   cloud_T(:,initial_point);
    
    rotation_angle  =   traj.yaw(1);
    
    % Define the desired overlap. In this case, 20 cm.
    overlap     =   0.2;

    loop        =   true;
    selection   =   true(1, cloud.Count);
    n_grouped   =   0;
    j           =   0;         
    sect_idx_prev     =   0;
%     figure; hold on; pcshow(cloud.Location); 
    while loop == true
        j = j + 1;
        % Remove already sectioned points to speed up the computations
        if j > 1
            sect_idx_prev    =   sections{j-1};
            selection(sect_idx_prev) = false;
            cloud_T         =   cloud_T_ids(2:end, selection);
            whole_cloud_ids =   cloud_T_ids(1, selection);
        end
        % Translate cloud to center in the base point
        translate_cloud     =   cloud_T - initial_point;
        translate_traj      =   traj_T - initial_point;
        % Rotate the yaw degrees CCW around a Z-axis through the base point
        rotation_matrix     =   rotz(rotation_angle);
        rotate_cloud        =   (rotation_matrix * translate_cloud);
        rotate_traj         =   (rotation_matrix * translate_traj);
        % Find the points within the interval and store indices in sections
        sec_idx             =   rotate_cloud(2,:) >= - sec_width;% & ...
    %                                  overlap > rotate_cloud(2,:) ;     
        traj_sect_idx       =   find(rotate_traj(2,:) >= - sec_width & ...
                                     overlap > rotate_traj(2,:) ); 
        % Use as next rotation point the closest traj point to the start of the
        % section and as the translation basis the closest ground point
        [~,i]               =   min(vecnorm(rotate_cloud - [0; - sec_width; 0]));
        [~,i2]              =   min(vecnorm(rotate_traj - [0; - sec_width; 0]));

        initial_point       =   cloud_T(:,i);
        rotation_angle      =   traj.yaw(i2);
        vertical_angle      =   traj.pitch(i2); 

        sec_idx     =   whole_cloud_ids(sec_idx);
        members     =   ismember(sec_idx, sect_idx_prev);
        sec_idx     =   sec_idx(~members);
        if j > 1 && numel(sec_idx) < numel(sections{j-1}) / 4
    %         if sum(members) ~= numel(ground_sect_idx)
                sec             =   cat(2,sections{j-1},sec_idx);
                sections{j-1}   =   unique(sec);
                t_sec           =   cat(2,traj_sec{j-1},traj_sect_idx);
                traj_sec{j-1}   =   unique(t_sec);
                n_grouped       =   n_grouped + length(sec_idx); 
    %         end
            j   =   j - 1;

            if n_grouped >= cloud.Count
                loop = false;
            end
        else        
            n_grouped       =   n_grouped + length(sec_idx);
            sections{j}     =   sec_idx;   
            traj_sec{j}     =   traj_sect_idx;
            traj_yaw(j)     =   rotation_angle;
            traj_pitch(j)   =   vertical_angle;
        end
    %     if isempty(ground_sect_idx)
    %         break
    %     end 

%     hold on; pcshow(cloud.Location(sections{j},:),rand(1,3)); 
    %  if ~isempty(traj_sect_idx)
    %     pcshow(traj.points((traj_sect_idx),:), 'y', 'MarkerSize', 200); hold on;
    %  end
    end

    SEC.sections        =   sections;
    SEC.traj_sections   =   traj_sec;
    SEC.traj_yaw        =   traj_yaw;
    SEC.traj_pitch      =   traj_pitch;
    SEC.sec_width       =   sec_width;
    
end


