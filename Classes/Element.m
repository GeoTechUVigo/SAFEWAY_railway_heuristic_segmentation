classdef Element
% Class with the geometrical parameters of a component
% Also has the function CheckElement to compare a point cloud of an
% element to its model Element.
%
%--------------------------------------------------------------------------
% INPUTS:
%
% dimensions : 1x3 meters. Dimensions of the object sorted by size. 
% 
% eigenvectors : 3x3  matrix with the vectors of its principal directions.
%               eigenvectors of pca() analysis.
% 
% eigenvalues : 1x3 for one unit of its principal directions.
%              eigenvalues of pca() analysis for one unit.
% 
% points : Nx3. Points of the model.
%
% toleranceDimensions : 1x3 meters. Tolerance of element dimensions.
% 
% toleranceeigenvectors : 1x3 meters. Tolerance of eigenvectors in degrees.
% 
% toleranceeigenvalues: 1x3 meters. Tolerance of eigenvalues.
% 
% tolerancePoints : numeric meters^2. Tolerance of distances^2
%                   between an element and its model cloud.
%                
% -------------------------------------------------------------------------
% OUTPUTS:
%
% Element class
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 28/12/2020
    
    properties        
        %% All the parameters must be double. Default: NaN
        
        % Dimensions in ABC (local edges) (eigenvectors) (positive)
        dimensions; % 1x3 meters
        
        % PCA
        eigenvectors; % 3x3 ortonormals
        eigenvalues; % 1x3 for one unit
        
        % Points of model
        points;
        
        % Tolerances (positives)
        toleranceDimensions; % 1x3 meters
        toleranceEigenvectors; % 1x3 angle in degrees
        toleranceEigenvalues; % 1x3 difference
        tolerancePoints; % % variance

    end
    
    methods
        %% Constructor
        function obj = Element(varargin)
            
            parser = inputParser;
            parser.CaseSensitive = true; 
               
            % Checking and saving the parameters    
            parser.addParameter('dimensions', NaN(1,3),@(x)validateattributes(x,{'double'}, {'nonnegative', 'size', [1,3]}));
            parser.addParameter('eigenvectors', NaN(3,3),@(x)validateattributes(x,{'double'}, {'size',[3,3]}));
            parser.addParameter('eigenvalues', NaN(1,3),@(x)validateattributes(x,{'double'}, {'size',[1,3]}));
            parser.addParameter('points',NaN(1,3),@(x)validateattributes(x,{'double'}, {'size',[NaN,3]}));
           
            parser.addParameter('toleranceDimensions', NaN(1,3),@(x)validateattributes(x,{'double'}, {'nonnegative', 'size', [1,3]}));
            parser.addParameter('toleranceEigenvectors', NaN(1,3),@(x)validateattributes(x,{'double'}, {'nonnegative', 'size', [1,3]}));
            parser.addParameter('toleranceEigenvalues', NaN(1,3),@(x)validateattributes(x,{'double'}, {'nonnegative', 'size', [1,3]}));
            parser.addParameter('tolerancePoints', NaN(1,1),@(x)validateattributes(x,{'double'}, {'size',[1,1]}));
                     
            parser.parse(varargin{:});

            obj.dimensions= parser.Results.dimensions;         
            obj.eigenvectors= parser.Results.eigenvectors;
            obj.eigenvalues= parser.Results.eigenvalues;  
            obj.points= parser.Results.points; 
            obj.toleranceDimensions= parser.Results.toleranceDimensions;           
            obj.toleranceEigenvectors= parser.Results.toleranceEigenvectors;
            obj.toleranceEigenvalues= parser.Results.toleranceEigenvalues;         
            obj.tolerancePoints= parser.Results.tolerancePoints; 
            
        end
        
        %% CheckElement
        function isElement = CheckElement(varargin)
            
            % Checking if all the parameters of the element points are
            % inside the dimensions of the element Element
            %--------------------------------------------------------------------------
            % INPUTS:
            %
            % points : Nx3. Points of the element that are going to be
            %          analyzed.
            % 
            % element : Element. Model.
            %                
            % -------------------------------------------------------------------------
            % OUTPUTS:
            %
            % isElement : logical. 1 if this varargin{2} is an element like
            %             varargin{1}
            %                           
            % -------------------------------------------------------------------------
            % Daniel Lamas Novoa.
            % Enxeñaría dos materiais, mecánica aplicada e construción.
            % Escola de enxeñería industrial
            % Grupo de xeotecnoloxía aplicada.
            % Universidade de Vigo.
            % 28/12/2020           
            
            parser = inputParser;
            parser.CaseSensitive = true; 
            parser.addRequired('points', @(x)validateattributes(x,{'double'}, {'nonempty', 'size', [NaN,3]})); % coordinates of the points of the element
            parser.addRequired('element', @(x)validateattributes(x,{'Element'}, {'nonempty'})); % model element
            parser.parse(varargin{:});
            points = parser.Results.points;
            element = parser.Results.element;
            
            isElement = true;
            
            %% PCA analysis 
            [eigenvectors,latents] = pcacov(cov(points));
            
            if numel(eigenvectors) ~= 9
                isElement = false;
                return;
            end
            
            %% eigenvectors
            for i = 1:3
                angle = acosd(dot(element.eigenvectors(:,i),eigenvectors(:,i))/(norm(element.eigenvectors(:,i))*norm(eigenvectors(:,i)))); % angle between vectors
                if angle > 90
                    angle = 180-angle;
                end
                if angle > element.toleranceEigenvectors(i)
                    isElement = false;
                    return;
                end
            end
            
            %% eigenvalues for one unit
            eigenvalues = zeros(3,1);
            for i =1:numel(latents)
               eigenvalues(i,1) = latents(i) /sum(latents); 
            end

            %% eigenvalues
 
            if any(abs(element.eigenvalues - eigenvalues') > element.toleranceEigenvalues)
                isElement = false;
                return;
            end            
    
            %% dimensions
            if any(abs(element.dimensions - range(points)) > element.toleranceDimensions)
                % checking dimensions orienting the points
                pointsOriented = points * eigenvectors; % points oriented by its pca
                
                [~, order]= max(eigenvectors, [], 1); % order of its principal directions in XYZ
                dimensions = element.dimensions(order); % dimensions in the order of its principal directions
                tolerances = element.toleranceDimensions(order); % same order
                
                if any(abs(dimensions - range(pointsOriented)) > tolerances) % Checking
                    isElement = false;
                    return;
                end
            end

            %% Distance between element.points and its closest points points
            
            % Considering just the top of the sign
            points = points(points(:,3) > min(element.points(:,3)),:);
            points = points - mean(points);
            points(:,3) = points(:,3) - max(points(:,3));
            
            [~,distances] = knnsearch(points, element.points); % from model to object
            [~,distances2] = knnsearch(element.points, points); % from object to model
            
%             figure; pcshow(element.points, 'g', 'MarkerSize', 100);
%             hold on; pcshow(points, 'r', 'MarkerSize', 100);
            
            if max(mean(distances.^2), mean(distances2.^2)) > element.tolerancePoints
                
                % rotating 180 degrees in Z
                rot180Z     = makehgtform('axisrotate',[0,0,1],180*pi/180); % transform matrix
                rot180Z     = rot180Z(1:3,1:3); % 3x3 rotation matrix
                
                points      = (rot180Z * points')';
                points      = points - mean(points);
                points(:,3) = points(:,3) - max(points(:,3));
                
                [~,distances] = knnsearch(points, element.points);
                [~,distances2] = knnsearch(element.points, points);
                
%                 figure; pcshow(element.points, 'g', 'MarkerSize', 100);
%                 hold on; pcshow(points, 'r', 'MarkerSize', 100);
                
                % Same operations but orienting the top of the element
                if max(mean(distances.^2), mean(distances2.^2)) > element.tolerancePoints
                   
                    points      = points * pca(points);
                    points      = points - mean(points);
                    points(:,1) = points(:,1) - max(points(:,1));
                    
                    elementOriented      = element.points * pca(element.points);
                    elementOriented      = elementOriented - mean(elementOriented);
                    elementOriented(:,1) = elementOriented(:,1) - max(elementOriented(:,1));
                                       
                    points = points(points(:,1) > min(elementOriented(:,1)),:); % recalculating its top
                    
                    [~,distances] = knnsearch(points, elementOriented);
                    [~,distances2] = knnsearch(elementOriented, points);
                    
%                 figure; pcshow(elementOriented, 'g', 'MarkerSize', 100);
%                 hold on; pcshow(points, 'r', 'MarkerSize', 100);

                    if max(mean(distances.^2), mean(distances2.^2)) > element.tolerancePoints
                        
                        % rotating 180 degrees in X because it is oriented
                        rot180Z     = makehgtform('axisrotate',[1,0,0],180*pi/180); % transform matrix
                        rot180Z     = rot180Z(1:3,1:3); % 3x3 transformation matrix

                        points      = (rot180Z * points')';
                        points      = points - mean(points);
                        points(:,1) = points(:,1) - max(points(:,1));
                        
                        [~,distances] = knnsearch(points, elementOriented);
                        [~,distances2] = knnsearch(elementOriented, points);
                        
%                         figure; pcshow(elementOriented, 'g', 'MarkerSize', 100);
%                         hold on; pcshow(points, 'r', 'MarkerSize', 100);
        
                        if max(mean(distances.^2), mean(distances2.^2)) > element.tolerancePoints
                            isElement = false;
                            return;
                        end
                    end
                end
            end
        end               
    end
end

