 function [cloudOut] = MergePointCloud_(varargin)
% Function to merge all pointClouds_.
% -------------------------------------------------------------------------
% INPUTS:
%
% varargin : cell array of pointCloud_
%                
% -------------------------------------------------------------------------
% OUTPUTS:
%
% cloudOut : pointCloud_. Cloud with all the points of the inpunt clouds
%            sorter by timeStamp.
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 28/12/2020

varargin = varargin{1};
% Variables to know which arguments the clouds have
haveIntensity = true;
haveTimeStamp = true;
haveColor     = true;
haveAngle     = true;
haveNormal    = true;

% Saving the number of points of each cloud + the previous
% cloud to filling the variables of cloudOut
numPoints(1) = 1;
for i = 1:numel(varargin)

    numPoints(i+1) = numPoints(i) + length(varargin{i}.Location);

    % Checking the arguments of each cloud. If one cloud does
    % not have an argument, cloudOut will not have it
    if isempty(varargin{i}.intensity)
        haveIntensity = false;
    end

    if isempty(varargin{i}.timeStamp)
        haveTimeStamp = false;
    end

    if isempty(varargin{i}.Color)
        haveColor = false;
    end

    if isempty(varargin{i}.angle)
        haveAngle = false;
    end

    if isempty(varargin{i}.Normal)
        haveNormal = false;
    end

end

% Initializing the variables to create the new cloud
points = zeros(numPoints(end) - 1, 3);  
sensor = uint8(zeros(numPoints(end) - 1,1));
if haveIntensity
    intensity = uint16(zeros(numPoints(end) - 1,1));
else
    intensity = [];
end
if haveTimeStamp
    timeStamp = zeros(numPoints(end) - 1,1);
else
    timeStamp = [];
end
if haveColor
    color     = uint8(zeros(numPoints(end) - 1,3));
else
    color = uint8.empty;
end
if haveAngle
    angle     = int8(zeros(numPoints(end) - 1,1));
else
    angle = [];
end
if haveNormal
    normal    = zeros(numPoints(end) - 1,1);
else
    normal = [];
end

% Filling the variables with the data of each input cloud
for i = 1:numel(varargin)

    points(numPoints(i):numPoints(i+1)-1,:)    = [varargin{i}.Location];
    sensor(numPoints(i):numPoints(i+1)-1,:)  = uint8(i);
    if haveIntensity
        intensity(numPoints(i):numPoints(i+1)-1,:) = [varargin{i}.intensity];
    end
    if haveTimeStamp
        timeStamp(numPoints(i):numPoints(i+1)-1,:) = [varargin{i}.timeStamp];
    end
    if haveColor
        color(numPoints(i):numPoints(i+1)-1,:)     = [varargin{i}.Color];
    end
    if haveAngle
        angle(numPoints(i):numPoints(i+1)-1,:)     = [varargin{i}.angle];
    end
    if haveNormal
        normal(numPoints(i):numPoints(i+1)-1,:)    = [varargin{i}.Normal];
    end

end

% Sorting data by timeStamp. It usually are sorted in point
% clouds
if haveTimeStamp
    [timeStamp,order]= sort(timeStamp, 'ascend');
    points = points(order,:);
    sensor = sensor(order,:);
    if haveIntensity
        intensity = intensity(order,:);
    end
    if haveColor
        color = color(order,:);
    end
    if haveAngle
        angle = angle(order,:);
    end
    if haveNormal
        normal = normal(order,:);
    end
end

% Generating th cloud
cloudOut = pointCloud_(points, 'intensity', intensity, 'timeStamp', timeStamp, 'Color', color, 'angle', angle, 'Normal', normal, 'sensor', sensor);
end

