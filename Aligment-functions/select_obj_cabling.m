function [ret_pts, cat_pts] = select_obj_cabling(cloud_sec, rails_sec)
% Function that identifies the cables located near a mast (return and feed 
% cables) and those above the rails (contact, catenary and droppers).  
% Inputs:
% -	cloud_input: pointCloud_ object containing the non-track points. 
% -	rails_input: 3D array containing the coordinates of the rails.
% Outputs:
% -	ret_pts: indices of the cables rough extraction.
% -	cat_pts: indices of the points corresponding to catenary, contact and
%            dropper cables.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Align and centre the cloud
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rails_pca   =   pca(rails_sec);
    rails       =   rails_sec * rails_pca;
    centre_pt   =   mean(rails);   
    
    cloud       =   align_cloud(cloud_sec,rails_pca); 
    cloud       =   translate_cloud(cloud,centre_pt); 
    
    cabling_pts =   cloud.Location;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute distance to a vertical plane parallel to the rails, set
    % on the origin, i.e., the centre of the rails
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    rep_normals     =   [0 1 0];
    rep_normals     =   repmat(rep_normals(1,:),length(cabling_pts),1);
    pt_plane_d      =   dot(cabling_pts, rep_normals,2);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Identify the objective cables (those corresponding to the 
    % trajectory). The objective catenary, contact, and droppers are the  
    % points in between rails and the return and feed wires are near them.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    cat_pts_i	=   (abs(pt_plane_d) <= 1.5);
    cat_pts     =   find(cat_pts_i);
    
    ret_pts_i   =   find(abs(pt_plane_d) <= 5 & abs(pt_plane_d) > 1.5);
    if var(sign(pt_plane_d(ret_pts_i))) > 0
        far_pts_i       =   abs(pt_plane_d) > 5;
        far_sign        =   mode(sign(pt_plane_d(far_pts_i)));
        % Keep as the return cable which is not at far cables side
        ret_pts_sign    =   sign(pt_plane_d(ret_pts_i));
        ret_pts_i       =   ret_pts_i(ret_pts_sign ~= far_sign);
    end
    ret_pts     =   (ret_pts_i);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Identify the remaining cables. Separate them using a kmeans division 
    % in the y-axes (perpendicular to the rails). The group of points 
    % closer to the objective cat_pts are catenated to them and the
    % remaining are catenated to ret_pts.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    rem_pts_i               =   true(length(cabling_pts), 1);
    rem_pts_i(ret_pts_i) 	=   false;
    rem_pts_i(cat_pts_i) 	=   false;
    if sum(rem_pts_i) > 5e3
        rem_pts_i       =   find(rem_pts_i);
        rem_pts         =	cabling_pts(rem_pts_i,:);
        if range(rem_pts(:,2)) > 2
            % There is a second set of cat/rem points
            divide_y    =   kmeans(rem_pts(:,2),2);
            y_1         =   mean(rem_pts(divide_y == 1,2));
            y_2         =   mean(rem_pts(divide_y == 2,2));
            % Cat points will be closer to rails
            [~, cat_id] =   min(abs([y_1, y_2]));
            add_cat_pts =   rem_pts_i(divide_y == cat_id);
            add_ret_pts =   rem_pts_i(divide_y ~= cat_id);
            % Concat arrays
            ret_pts     =   [ret_pts; (add_ret_pts)];
            cat_pts     =   [cat_pts; (add_cat_pts)];
        end
    end  
    
    
