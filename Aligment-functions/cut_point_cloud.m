function [cut_cloud] = cut_point_cloud(cloud, traj, distance)
% Function that aligns a point cloud to the trajectory and returns the
% section of the cloud that is within a perpendicular distance from the
% trajectory.
%Inputs: 
% - cloud: Original pointCloud_ class object
% - traj: Object of trajectory class
% - distance: Desired width of the output cloud
%Outputs: 
%  - cut_cloud: Section of the cloud as a pointCloud_ class object
 
cut_idx     =   zeros(1,cloud.Count); % initialise all zeros - false
cloud_T     =   transpose(cloud.Location);

% figure; pcshow(cloud.Location, cloud.intensity)
% hold on; pcshow(ground_traj.points,'r','markersize', 80) 
for i = 1:3:traj.count-4
    initial_point       =   traj.points(i,:);
    rotation_angle      =   traj.yaw(i);
    % Translate cloud to center in the base point
    translate_cloud     =   cloud_T - initial_point';
    translate_traj      =   traj.points - initial_point;
    % Rotate the yaw degrees CCW around a Z-axis through the base point
    rotation_matrix     =   rotz(rotation_angle);
    rotate_cloud        =   (rotation_matrix * translate_cloud);
    rotate_traj         =   (rotation_matrix * translate_traj');
    % Use as next point the following 2 trajectory point to allow some overlap 
    next_point_y        =   rotate_traj(2,i+4) - rotate_traj(2,i);
    % Create a section of the cloud between the two points Y and within
    % the defined X distance
    cloud_section       =   (rotate_cloud(2,:) >= next_point_y & rotate_cloud(2,:) < 0 ...
                            & rotate_cloud(1,:)> - distance/2 & rotate_cloud(1,:)< distance/2);
    cut_idx             =   cut_idx + cloud_section;

% figure; hold on; pcshow(rotate_cloud'); pcshow(rotate_traj','y','markersize', 80) 
% hold on; pcshow(cloud.Location(cloud_section,:), rand(1,3))%; xlabel('x axes','color','w')
end
    
% Convert the cumulative indices into logical (0-> false; 1,2...-> true)
cut_idx     =   find(cut_idx);    
% Save into pointCloud_ object
cut_cloud   =   select(cloud, cut_idx);
          
end

