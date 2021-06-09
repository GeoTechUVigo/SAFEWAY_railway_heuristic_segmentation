function [model] = GenerateElements(pathModels)
% Generate models of elements using its clouds and/or its data. 
% The elements are Element.
% -------------------------------------------------------------------------
% INPUTS:
%
% pathModels : point cloud models path
%
% -------------------------------------------------------------------------
% OUTPUTS:
%
% model : cell array of Element.
%                           
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 28/12/2020

load([pathModels, 'trafficLightSigns.mat']);
load([pathModels, 'trafficLightSign2.mat']);
load([pathModels, 'trafficLightSigns3.mat']);
load([pathModels, 'stoneSignal.mat']);
load([pathModels, 'stoneSignal2.mat']); % light sign
load([pathModels, 'stoneSignal3.mat']);

model.bigSignalModel     = Element('dimensions', [0.34,1.1,5], 'toleranceDimensions', [0.14,0.3,1],'eigenvectors', [0,0,1;0,1,0;1,0,0], 'toleranceEigenvectors', [7,10,10], 'eigenvalues', [0.96,0.03,0.01], 'toleranceEigenvalues', [0.03,0.02,0.01]); % without point cloud model because the number of signs it is not alway the same. It is done to not segment poles without any signs (because eigenvalues ranges).
model.trafficLightModel  = Element('dimensions', [2.0,0.65,NaN],'toleranceDimensions',[1,0.2,NaN],'eigenvectors', [0,NaN,NaN;0,NaN,NaN;1,NaN,NaN], 'toleranceEigenvectors', [15,NaN,NaN], 'eigenvalues', [0.92,0.07,0.01], 'toleranceEigenvalues', [0.05,0.05,0.05], 'points', trafficLightSigns ,'tolerancePoints', 0.03); % 0.007
model.trafficLightModel2 = Element('dimensions', [2.0,0.65,NaN],'toleranceDimensions',[1,0.2,NaN],'eigenvectors', [0,NaN,NaN;0,NaN,NaN;1,NaN,NaN], 'toleranceEigenvectors', [15,NaN,NaN], 'eigenvalues', [0.92,0.07,0.01], 'toleranceEigenvalues', [0.07,0.07,0.05], 'points', trafficLightSign2 ,'tolerancePoints', 0.03); % 0.3
model.trafficLightModel3 = Element('dimensions', [2.0,0.65,NaN],'toleranceDimensions',[0.6,0.2,NaN],'eigenvectors', [0,NaN,NaN;0,NaN,NaN;1,NaN,NaN], 'toleranceEigenvectors', [15,NaN,NaN], 'eigenvalues', [0.92,0.07,0.01], 'toleranceEigenvalues', [0.07,0.07,0.05], 'points', trafficLightSigns3 ,'tolerancePoints', 0.03); % 0.3
model.stoneSignalModel   = Element('dimensions', [0.25,0.35,NaN],'toleranceDimensions', [0.1,0.1,NaN],'eigenvectors', [0,0,1;0,1,0;1,0,0], 'toleranceEigenvectors', [15,25,25], 'eigenvalues', [NaN,NaN,NaN], 'toleranceEigenvalues', [NaN,NaN,NaN], 'points', stoneSignal, 'tolerancePoints', 0.001); % detectadas a 0.00078
model.stoneSignalModel3  = Element('dimensions', [0.25,0.37,NaN],'toleranceDimensions', [0.1,0.1,NaN],'eigenvectors', [0,0,1;0,1,0;1,0,0], 'toleranceEigenvectors', [15,25,25], 'eigenvalues', [NaN,NaN,NaN], 'toleranceEigenvalues', [NaN,NaN,NaN], 'points', stoneSignal3, 'tolerancePoints', 0.0015);
model.light              = Element('dimensions', [0.7,0.8,NaN],'toleranceDimensions', [0.15,0.15,NaN],'eigenvectors', [0,NaN,NaN;0,NaN,NaN;1,NaN,NaN], 'toleranceEigenvectors', [15,NaN,NaN], 'eigenvalues', [NaN,NaN,NaN], 'toleranceEigenvalues', [NaN,NaN,NaN], 'points', stoneSignal2, 'tolerancePoints', 0.01);

model.mastNoBracketModel = Element('dimensions', [1.2,0.75,10], 'eigenvectors', [0,NaN,NaN;0,NaN,NaN;1,NaN,NaN], 'eigenvalues', [0.85,NaN, NaN], 'toleranceDimensions', [1.2,0.75,4], 'toleranceEigenvectors', [20,NaN,NaN], 'toleranceEigenvalues', [0.15,NaN,NaN]); % antes 8+2 de alto en Y 0.75+0.75
model.cableModel         = Element('dimensions', [NaN,0.3,NaN], 'eigenvectors', [NaN,NaN,NaN;NaN,NaN,NaN;NaN,NaN,NaN], 'eigenvalues', [NaN,NaN, NaN], 'toleranceDimensions', [NaN,NaN,NaN], 'toleranceEigenvectors', [NaN,NaN,NaN], 'toleranceEigenvalues', [NaN,NaN,NaN]);
model.dropperModel       = Element('dimensions', [NaN,1,NaN], 'eigenvectors', [0,NaN,NaN;0, NaN, NaN;1, NaN, NaN], 'eigenvalues', [NaN,NaN, NaN], 'toleranceDimensions', [NaN,1,NaN], 'toleranceEigenvectors', [NaN,NaN,NaN], 'toleranceEigenvalues', [NaN,NaN,NaN]);
model.railModel          = Element('dimensions', [NaN,0.6,0.1], 'eigenvectors', [NaN,NaN,NaN;NaN, NaN, NaN;NaN, NaN, NaN], 'eigenvalues', [NaN,NaN, NaN], 'toleranceDimensions', [NaN,NaN,NaN], 'toleranceEigenvectors', [NaN,NaN,NaN], 'toleranceEigenvalues', [NaN,NaN,NaN]); % 'dimensions', [NaN,0.3,0.1]
model.railPairModel      = Element('dimensions', [NaN,1.74,0.3], 'eigenvectors', [NaN,NaN,NaN;NaN, NaN, NaN;NaN, NaN, NaN], 'eigenvalues', [NaN,NaN, NaN], 'toleranceDimensions', [NaN,0.02,NaN], 'toleranceEigenvectors', [NaN,NaN,NaN], 'toleranceEigenvalues', [NaN,NaN,NaN]);

end

