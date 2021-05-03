function [rail_tips]     =   detect_rail_tips(varargin)
% Function that identifies the tips of the rails and divides the railways
% into clusters containing one rail only.
% Inputs:
% - rail_cloud: pointCloud_ object of rails (from a rough segmentation)    
% - traj_point: point of the trajectory used for translating the cloud   
% - traj_yaw: yaw of the trajectory point for a rotation in Z
% - traj_pitch: pitch of the trajectory point for a rotation in X
% Outputs:
% - rail_tips: coordinates of the tips on the original rail_cloud    
% - new_cluster_id: vector with a cluster id for each point. The points 
%   located further than 1.1m from the trajectory are indexed with zero  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


rail_cloud  =   varargin{1};
traj_point  =   varargin{2};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Determine the angle that the trajectory forms with the y and z axes.
% Centre the cloud and trajectory and rotate them to align them to be
% perpendicular to the x axis.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Determine angle with the y axis -> theta 
traj_vec    =   traj_point(end,:) - traj_point(1,:);
y_axes      =   [0 1 0];

cross_p     =   cross(y_axes,traj_vec,2);
dot_p       =   dot(y_axes,traj_vec,2);
ThetaInDeg  =   atan2d(vecnorm(cross_p),dot_p);
% Determine angle with the y axis -> Phi 
z_axes      =   [0 0 1];    
cross_p     =   cross(z_axes,traj_vec,2);
dot_p       =   dot(z_axes,traj_vec,2);
PhiInDeg    =   atan2d(vecnorm(cross_p),dot_p);

% Centre the cloud and the trajectory
tras_pt     =   traj_point(1,:);
traj_pts    =   traj_point - tras_pt;
cloud_pts   =   rail_cloud.Location - mean(rail_cloud.Location);

% Define the rotation matrices
traj_yaw    =   ThetaInDeg;
traj_pitch  =   90  - PhiInDeg;
    
% % OLD METHOD %%%%%%%%%%%%%%%
% if length(varargin) == 4
%     traj_yaw    =   varargin{3};
%     traj_pitch  =   varargin{4};
% end
% % OLD METHOD %%%%%%%%%%%%%%%%  


rot_x       =   rotx(traj_pitch);
rot_z       =   rotz(traj_yaw); 
traj_pts    =   (traj_pts * rot_z) * rot_x;
% figure; pcshow(cloud_pts,'w', 'markersize', 50);hold on; pcshow(traj_pts, 'r', 'markersize', 100)
% Check if the rotated angles were the correct ones. A good rotation will 
% give a line along to the y-axis.
if range(traj_pts(:,1)) > 0.2 % 
    traj_yaw    =   180 - ThetaInDeg;
    rot_x       =   rotx(traj_pitch);
    rot_z       =   rotz(traj_yaw); 
    traj_pts    =   traj_point - tras_pt;
    traj_pts    =   (traj_pts * rot_z) * rot_x;
    if range(traj_pts(:,1)) > 0.2 % 
        flag = true;
    end
end
    
% Rotate the cloud
cloud_pts   =   (cloud_pts * rot_z) * rot_x;
 
% hold on; pcshow(cloud_pts,'y', 'markersize', 50); hold on; pcshow(traj_pts, 'b', 'markersize', 100)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sort points of the rotated cloud in Y and select the unique points in X.
% Then perform a histcount classification to remove isolated points.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cloud_pts_num   =   [(1:rail_cloud.Count)', cloud_pts];
data            =   cloud_pts_num;
data_sorted     =   sortrows(data,3);    

 
[~,idx,~]   =   unique(data_sorted(:,2));
data_uniq   =   data_sorted(idx,:);

% Remove points that are isolated in Z
[Z_counts, ~, Z_bin]    =   histcounts(data_uniq(:,4),'BinWidth', 0.05);

keep_Z                  =   find(Z_counts > 1+length(data_uniq(:,1)) * 0.01);
keep_Z                  =   ismember(Z_bin, keep_Z);
data_uniq               =   data_uniq(keep_Z,:);

% max(Z_counts)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Use the XZ data to find the peaks of the cloud applying a findpeaks
% method. Minimum peak height defined as the third quartile. Check the
% variance of the peaks found. If the variance is low, choose the mean of
% the points. But if the variance is high, separate them into two groups as
% there can be more than one rail in the area (crossing rails). Do this
% process twice to consider up to 4 rails.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
XZ          =   [data_uniq(:,2),data_uniq(:,4)];
 
min_height  = mean([max(XZ(:,2)), mean(XZ(:,2))]);
min_height  = mean([max(XZ(:,2)), min_height]);


if numel(XZ(:,1)) < 3 
    [~, max_id] =   max(rail_cloud.Location(:,3));
    rail_tips   =   rail_cloud.Location(max_id,:);
else
    [Zloc, Xloc]   =   findpeaks(XZ(:,2),XZ(:,1),...
                       'MinPeakHeight',min_height);
    if isempty(Zloc)
        [~, max_id] =   max(rail_cloud.Location(:,3));
        rail_tips   =   rail_cloud.Location(max_id,:);
    else        
        % Standard deviation of the height peaks            
        std_Xloc        =   std(Xloc);
        if std_Xloc > 0.07 % This means that there are two or more rails 
            % Divide into two and pick two tips
            kmeas_X     =   kmeans(Xloc, 2);
            Xloc_final  =   [];
            Zloc_final  =   [];

        %%%%% LEFT 1
            Xloc_1      =   Xloc(kmeas_X == 1);
            Zloc_1      =   Zloc(kmeas_X == 1); 
            std_Xloc_1  =   std(Xloc_1);    
            if std_Xloc_1 < 0.07        
                mean_val     =   (max(Xloc_1) + min(Xloc_1)) / 2;
                [~, mid_i]  =   min(abs(Xloc_1 - mean_val));
                Xloc_final        =   Xloc_1(mid_i);
                Zloc_final        =   Zloc_1(mid_i);
            else % Divide again
                kmeas_X1     =   kmeans(Xloc_1, 2);
                % Left 11
                Xloc_11      =   Xloc_1(kmeas_X1 == 1);  
                Zloc_11      =   Zloc_1(kmeas_X1 == 1);       
                std_Xloc_11  =   std(Xloc_11);
                if std_Xloc_11 < 0.07      
                   mean_val     =   (max(Xloc_11) + min(Xloc_11)) / 2;
                   [~, mid_i]   =   min(abs(Xloc_11 - mean_val));
                   Xloc_final   =   Xloc_11(mid_i);
                   Zloc_final   =   Zloc_11(mid_i);   
                end
                % Right 12
                Xloc_12      =   Xloc_1(kmeas_X1 == 2);  
                Zloc_12      =   Zloc_1(kmeas_X1 == 2);       
                std_Xloc_12  =   std(Xloc_12);
                if std_Xloc_12 < 0.07            
                   mean_val     =   (max(Xloc_12) + min(Xloc_12)) / 2;
                   [~, mid_i]   =   min(abs(Xloc_12 - mean_val));
                   Xloc_final   =   [Xloc_final; Xloc_12(mid_i)];
                   Zloc_final   =   [Zloc_final; Zloc_12(mid_i)];        
                end
            end

        %%%%% RIGHT 2
            Xloc_2      =   Xloc(kmeas_X == 2);
            Zloc_2      =   Zloc(kmeas_X == 2); 
            std_Xloc_2  =   std(Xloc_2);     
            if std_Xloc_2 < 0.07        
                mean_val     =   (max(Xloc_2) + min(Xloc_2)) / 2;
                [~, mid_i]  =   min(abs(Xloc_2 - mean_val));
                Xloc_final        =   [Xloc_final; Xloc_2(mid_i)];
                Zloc_final        =   [Zloc_final; Zloc_2(mid_i)];
            else % Divide again
                kmeas_X2     =   kmeans(Xloc_2, 2);
                % Left 21
                Xloc_21      =   Xloc_2(kmeas_X2 == 1);  
                Zloc_21      =   Zloc_2(kmeas_X2 == 1);       
                std_Xloc_21  =   std(Xloc_21);
                if std_Xloc_21 < 0.07  
                    mean_val     =   (max(Xloc_21) + min(Xloc_21)) / 2;
                    [~, mid_i]   =   min(abs(Xloc_21 - mean_val));
                    Xloc_final   =   [Xloc_final; Xloc_21(mid_i)];
                    Zloc_final   =   [Zloc_final; Zloc_21(mid_i)];   
                end
                % Right 12
                Xloc_22      =   Xloc_2(kmeas_X2 == 2);  
                Zloc_22      =   Zloc_2(kmeas_X2 == 2);       
                std_Xloc_22  =   std(Xloc_22);
                if std_Xloc_22 < 0.07
                    mean_val     =   (max(Xloc_22) + min(Xloc_22)) / 2;
                    [~, mid_i]   =   min(abs(Xloc_22 - mean_val));
                    Xloc_final   =   [Xloc_final; Xloc_22(mid_i)];
                    Zloc_final   =   [Zloc_final; Zloc_22(mid_i)];        
                end
            end
            Xloc = Xloc_final;
            Zloc = Zloc_final;

        else % In this case, there is only one rail
           mean_val     =   (max(Xloc) + min(Xloc)) / 2;
           [~, mid_i]   =   min(abs(Xloc - mean_val));
           Xloc         =   Xloc(mid_i);
           Zloc         =   Zloc(mid_i);
        end
        
        if ~isempty(Xloc)
            % Recover the location of the tips in the original cloud
            tips        =   ismember(XZ,  [Xloc,Zloc], 'rows');
            rail_tips   =   data_uniq(tips,1);
            rail_tips   =   rail_cloud.Location(rail_tips,:);
        else
            rail_tips   = [];
        end
    end
end

end % function
% 
% figure; findpeaks(XZ(:,2),XZ(:,1),'MinPeakHeight',min_height);
% hold on; plot(Xloc, Zloc, 'ko', 'markerfacecolor', 'r','markersize', 8)
% hold on; xlabel('X [m]')
% hold on; ylabel('Z [m]')
