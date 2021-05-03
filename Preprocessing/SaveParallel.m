function [] = SaveParallel(varargin)
% Function to save inside of a parallel loop for
% Saving components and status

% components = varargin{2};
% status = varargin{3};
% save([varargin{1}], 'components', 'status');

status = varargin{2};
save([varargin{1}],'status');
end

