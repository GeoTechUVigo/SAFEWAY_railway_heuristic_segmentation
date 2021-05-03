function traj_interval = assign_traj_to_sections(SEC, cloud_traj)
% Function that provides an interval of trajectory points for each section
% in SEC.
% Inputs:
%  - SEC   - struct containing for a point cloud: (1) sections indices; 
%           (2) track_id (tack point indices in each of the sections); 
%           (3) traj_yaw and (4) traj_pitch (trajectory yaw and pitch
%            identified for each section)
%  - cloud_traj  - trajectory object
% Output:
%  - traj_interval - 2D array containing the start and end point indices 
%                   of the trajectory in each section 

       
    traj_interval       =   zeros(numel(SEC.sections),2);
    % Account for the sections in which no trajectory points lie
    empty               =   cellfun(@isempty, SEC.traj_sections);
    not_empty_ids       =   find(~empty);
    not_empty_intervals =   SEC.traj_sections(~empty);

    actual_pt           =   not_empty_intervals{1}(1);
    next_pt             =   actual_pt + 1;
    next_pt_id          =   find(cellfun(@(x) sum(x == next_pt), SEC.traj_sections));
    traj_interval(1,:)  =   [actual_pt, next_pt];
    
    points_considered   =   cat(2, SEC.traj_sections{:}); 
    for i = 2:numel(SEC.sections)
        % Determine the points corresponding to the section
        if length(SEC.traj_sections{i}) > 1
            % Simplify points in one section
            pts         =   cloud_traj.points(SEC.traj_sections{i},:);
            pts_p1      =   [pts(end,:); pts(1:end-1,:)];
            norms       =   vecnorm(pts - pts_p1,2,2);
            if any(norms > 1)
                SEC.traj_sections{i}    =   SEC.traj_sections{i}(norms > 1);
                points_considered       =   cat(2, SEC.traj_sections{:}); 
            end
            
            next_pt     =   SEC.traj_sections{i}(end);
            actual_pt   =   SEC.traj_sections{i}(1); 
        else
            prev_pt      =    actual_pt;
            actual_pt    =    SEC.traj_sections{i};     
            if ~empty(i) && actual_pt ~= prev_pt
                next_pt     =   actual_pt + 1;
            elseif empty(i) 
                actual_pt   =  traj_interval(i-1,2);
                next_pt     =  actual_pt + 1;
            end   
        end
        if next_pt < max(points_considered)
            traj_interval(i,:)   =   [actual_pt, next_pt]; 
        else
            traj_interval(i,:)   =   [max(points_considered)-1, max(points_considered)];  
        end
        
    end
end