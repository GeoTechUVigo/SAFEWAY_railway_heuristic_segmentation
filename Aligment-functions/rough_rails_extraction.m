function RR = rough_rails_extraction(SEC,cloud,traj)
% Function that provides a rough segmentation of the rails in a point cloud
% and identifies the top points along them.
% Inputs:
%  - SEC   - struct containing for a group of sections: (1) sections (all 
%            the points); (2) track_id (tack points of the section); (3)
%            traj_interval (two trajectory points corresponding to the 
%            section); (4) traj_yaw and (5) traj_pitch (trajectory yaw and
%            pitch identified for each section); (6) track_id and 
%            notTrack_id, track and non track points for each section.
%  - cloud - pointCloud_ object to which the sections refer
%  - traj  - trajectory object to which the sections refer
% Output:
%  - RR - struct containing (1) rail_1_rough_id & rail_2_rough_id (the
%         indices of the rough rails in each section); (2) tips_1 and 
%         tips_2 (the tips of the rails in each section); and (3) nTips
%         (the number of tips in each section)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    RR.rail_1_rough_id     =   cell(size(SEC.sections));
    RR.rail_2_rough_id     =   cell(size(SEC.sections)); 
    RR.tips_1   =   cell(size(SEC.sections));
    RR.tips_2	=   cell(size(SEC.sections));
    RR.nTips	=   zeros(numel(SEC.sections),2); 

    % Rasterise the ground using the provided grid size
    track_ids   =   cat(2,SEC.track_id{:});
    track       =   select(cloud, track_ids);    
    rough_rails_0   =   Rails_raster_detect(track, 0.04);
    rough_rails_0   =   track_ids(rough_rails_0);
    
    %% Start loop over sections to determine the rail tips
    for i = 1:numel(SEC.sections)  
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Recover the trajectory and cloud pointscorresponding to this 
        % section as well as the rough rails points
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        v1          =   traj.points(SEC.traj_interval(i,1),:);
        v2          =   traj.points(SEC.traj_interval(i,2),:); 
        
        sec_pts     =   SEC.track_id{i};     % (Whole cloud indices)   
        rough_sec   =   rough_rails_0(ismember(rough_rails_0, sec_pts));        
        
        if length(rough_sec) < 5;       continue;        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Remove points inmediate below the trajectory or too far away and
        % store the individual rails
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rough_sec_xyz   =   cloud.Location(rough_sec,:);
        [D_sec, s_sec]  =   point_to_line_distance(rough_sec_xyz(:,1:2),...
                            v1(1:2), v2(1:2));
        rough_sec       =   rough_sec(D_sec > 0.5 & D_sec < 1.2 );
        s_sec           =   s_sec(D_sec > 0.5 & D_sec < 1.2);

        rail_1_ids  =   rough_sec(s_sec == -1);
        rail_2_ids  =   rough_sec(s_sec == 1);     
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % For each rail section find the tips and use them to daw a line  
        % along the rail with the same direction as the trajectory. 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        if ~isempty(rail_1_ids)
            RR.rail_1_rough_id{i}  =   rail_1_ids;
            rail_1    =   select(cloud, rail_1_ids);  
            RR.tips_1{i}  =   detect_rail_tips(rail_1,...
                              traj.points(SEC.traj_interval(i,:),:));   
            RR.nTips(i,1) 	=   size(RR.tips_1{i},1); 
        end
        if ~isempty(rail_2_ids)
            RR.rail_2_rough_id{i}  =   rail_2_ids;    
            rail_2    =   select(cloud, rail_2_ids);    
            RR.tips_2{i}  =   detect_rail_tips(rail_2, ...
                                    traj.points(SEC.traj_interval(i,:),:));
            RR.nTips(i,2)    =   size(RR.tips_2{i},1); 
        end
    end
end


%     figure
% hold on; pcshow(cloud.Location(rough_rails_0,:), 'w')
% not_rough = true(1, length(track_ids));
% not_rough(rough_rails_0) = false;
% hold on; pcshow(cloud.Location(not_rough,:),clour.intensity(not_rough))
%         hold on; pcshow(cloud.Location(RR.rail_1_rough_id{i},:),'r')
%         hold on; pcshow(cloud.Location(RR.rail_2_rough_id{i},:),'y')
%         if ~isempty(RR.tips_1{i})
%             hold on; pcshow(RR.tips_1{i},'w','markersize',200)
%         end
%         if ~isempty(RR.tips_2{i})
%             hold on; pcshow(RR.tips_2{i},'r','markersize',200)
%         end