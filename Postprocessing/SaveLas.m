function [] = SaveLas(point_cloud, path)
% Funtion that generates a structure taht LASwrite function can save into a
% .las
% The information to correctly write all the structure is here:
% https://www.asprs.org/a/society/committees/standards/asprs_las_format_v12.pdf
% =========================================================================
% INPUTS:
% point_cloud
%
% path: char or string. Path where the .las will be write.
% =========================================================================
%% Structure
%% Header
structure.header.file_signature = 'LASF';
structure.header.file_source_id = 0;
structure.header.global_encoding_gps_time_type = 1;
structure.header.global_encoding_reserved = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
structure.header.project_id_1 = 0;
structure.header.project_id_2 = 0;
structure.header.project_id_3 = 0;
structure.header.project_id_4 = [0;0;0;0;0;0;0;0];
structure.header.version_major = 1;
structure.header.version_minor = 2;
structure.header.system_identifier = 'LAStools (c) by rapidlasso GmbH';
structure.header.generating_software = 'lasmerge (version 210418)';
structure.header.file_creation_doy = 0;
structure.header.file_creation_year = 0;
structure.header.header_size = 227;
structure.header.offset_to_data = 227;
structure.header.n_variable_length_records = 0;
structure.header.point_data_format_id = 3;
structure.header.point_data_record_length = 34;
structure.header.n_point_records = size(point_cloud.Location,1);
structure.header.n_points_by_return = [0;0;0;0;0];
structure.header.x_scale_factor = 0.001;
structure.header.y_scale_factor = 0.001;
structure.header.z_scale_factor = 0.001;
structure.header.x_offset = round(mean(point_cloud.Location(:,1)));
structure.header.y_offset = round(mean(point_cloud.Location(:,2)));
structure.header.z_offset = round(mean(point_cloud.Location(:,3)));
structure.header.max_x = max(point_cloud.Location(:,1));
structure.header.min_x = min(point_cloud.Location(:,1));
structure.header.max_y = max(point_cloud.Location(:,2));
structure.header.min_y = min(point_cloud.Location(:,2));
structure.header.max_z = max(point_cloud.Location(:,3));
structure.header.min_z = min(point_cloud.Location(:,3));

%% Record
structure.record.x = double(point_cloud.Location(:,1));
structure.record.y = double(point_cloud.Location(:,2));
structure.record.z = double(point_cloud.Location(:,3));
if ~isempty(point_cloud.intensity)
    structure.record.intensity = uint16(point_cloud.intensity);
else
    structure.record.intensity = uint16(zeros(size(structure.record.x)));
end

structure.record.return_number = uint8(zeros(size(structure.record.x)));
structure.record.number_of_returns = uint8(zeros(size(structure.record.x)));
structure.record.scan_direction_flag = false(size(structure.record.x));
structure.record.flightline_edge_flag = false(size(structure.record.x));
structure.record.classification = uint8(zeros(size(structure.record.x)));
structure.record.classification_synthetic = false(size(structure.record.x)); 
structure.record.classification_keypoint = false(size(structure.record.x));
structure.record.classification_withheld = false(size(structure.record.x));
structure.record.scan_angle = int8(zeros(size(structure.record.x)));
structure.record.user_data = uint8(zeros(size(structure.record.x)));
structure.record.point_source_id = uint16(zeros(size(structure.record.x)));

if ~isempty(point_cloud.timeStamp)
    structure.record.gps_time = double(point_cloud.timeStamp);
else
    structure.record.gps_time = zeros(size(structure.record.x));
end
if ~isempty(point_cloud.Color)
    structure.record.red = uint16(point_cloud.Color(:,1));
    structure.record.green = uint16(point_cloud.Color(:,2));
    structure.record.blue = uint16(point_cloud.Color(:,3));
else
    structure.record.red = uint16(zeros(size(structure.record.x)));
    structure.record.green = uint16(zeros(size(structure.record.x)));
    structure.record.blue = uint16(zeros(size(structure.record.x)));
end

%% Save
LASwrite(structure,char(path));

end

