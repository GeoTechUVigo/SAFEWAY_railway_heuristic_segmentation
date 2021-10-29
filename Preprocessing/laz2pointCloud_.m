function [ cloud ] = laz2pointCloud_( lasFile )

%Convert a cloud .las into pointCloud_ class object. 

%Input: lasFile: File path .las
%Output: cloud: pointCloud_ object

% cloud = lasdata(lasFile);
% cloud = pointCloud_([cloud.x , cloud.y, cloud.z], 'intensity', cloud.intensity, 'timeStamp', cloud.gps_time, 'angle', cloud.scan_angle);

lasReader = lasFileReader(lasFile);
ptCloud = readPointCloud(lasReader);
cloud = pointCloud_(ptCloud.Location, 'intensity', ptCloud.Intensity);

% with color
% color = [uint8(double(cloud.red)/ 65025 * 255), uint8(double(cloud.green)/ 65025 * 255), uint8(double(cloud.blue)/ 65025 * 255)];
% cloud = pointCloud_([cloud.x , cloud.y, cloud.z], 'Color', color, 'intensity', cloud.intensity, 'timeStamp', cloud.gps_time, 'angle', cloud.scan_angle);

end

