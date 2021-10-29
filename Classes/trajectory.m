classdef trajectory
%Objeto trajectory para almacenar la trayectoria del sistema de mapeado. 
%traj = trajectory(traj_path) lee un archivo que contenga los datos de una
%trayectoria (el archivo que se leía aquí consistía de un punto por cada
%fila, con 7 datos separados por coma) y crea el objeto a partir de esos
%datos.

%traj = trajectory(points, timeStamp, roll, pitch, yaw) crea la trayectoria
%a partir de variables (Todas ellas deberían ir en este orden y tener el
%mismo número de puntos)
    
    properties
        points; %Puntos de la trayectoria (x,y,z)
        timeStamp; %Time stamp de cada uno de los puntos. 
        roll; %Roll del vehículo en cada uno de los puntos.
        pitch; %Pitch del vehículo en cada uno de los puntos. 
        yaw; %Yaw del vehículo en cada uno de los puntos. 
        count; %Número de puntos. 
    end
    
    methods
        function this = trajectory(varargin) %Constructor de la clase.
            narginchk(1, 5);
            if (numel(varargin) == 1) %Create trajectory from txt file.
                trajectory_file = varargin{1};
                fileID = fopen(trajectory_file);
                C = textscan(fileID, '%f %f %f %f %f %f %f', 'Delimiter', '\t');
                this.points = [C{2}, C{3}, C{4}];
                this.roll = C{5}; 
                this.pitch = C{6}; 
                this.yaw = C{7};
                this.timeStamp = C{1}; 
                
            elseif (numel(varargin) == 2) %Create trajectory from data: points and timestamp.
                if (size(varargin{1}, 2) == 3)
                    this.points = varargin{1}; 
                    this.timeStamp = varargin{2};
                elseif (size(varargin{1}, 2) == 1)
                    this.points = varargin{2};
                    this.timeStamp = varargin{1}; 
                end
            elseif (numel(varargin) == 5)
               this.points = varargin{1}; 
               this.timeStamp = varargin{2}; 
                this.roll = varargin{3};
                this.pitch = varargin{4};
                this.yaw = varargin{5};
            end
            this.count = size(this.points,1);
        end
        function traj_out = select(this, idx)
            
            try
            traj_out = trajectory(this.points(idx,:), this.timeStamp(idx,:), ...
                        this.roll(idx), this.pitch(idx), this.yaw(idx));
            catch
            traj_out = trajectory(this.points(idx,:), this.timeStamp(idx,:));
            end
        end
        function traj_out = rotate_traj_x(this, varargin)
            %select Select points specified by index.
            %
            %  ptCloudOut = select(ptCloud, indices) returns a pointCloud
            %  object that rotates the points of the original cloud

            narginchk(2, 3);
            
            if nargin == 2
                angle_rot = varargin{1};
                validateattributes(angle_rot, {'numeric','double'}, ...
                    {'nonsparse','finite'}, mfilename, 'angle_rot');
            else
                % Subscript syntax is only for organized point cloud
                if ndims(this.Location) ~= 3
                    error(message('vision:pointcloud:organizedPtCloudOnly'));
                end
                    
            end
            
            % Obtain the subset for every property
            pts     = (rotx(angle_rot) * (this.points'))';
            
            
            traj_out = trajectory(pts, this.timeStamp, ...
                        this.roll, this.pitch, this.yaw);
        end
    end  
end

