function [sections] = Sectioning_trajectory(traj)
% Function to split the traj in sections by length and curvature.

%% Parameters for sections
step         = 100; % max length of each sec
maxCurvature = 5; % max curvature of each section in meters. It is the max range in y
width        = 20;
overlap      = 5; % overlap between sections

sections = [];

%% 
firstPoint      = 1;

%% Sections
allTrajDone = true;
j = 0;
lastPoint = firstPoint;
while lastPoint ~= length(traj.points)
    j = j + 1;
    
    lastPoint = find(pdist2(traj.points, traj.points(firstPoint,:)) < step, 1, 'last');
    
     if ~isempty(lastPoint)
        idsTrajSec       = (firstPoint:lastPoint)'; % ids of the traj between first and last point of this section
        trajSecPointsPca = traj.points(idsTrajSec,:) - mean (traj.points(idsTrajSec,:)); % traj sec points centered
        pcaTraj          = PcaFlattering(trajSecPointsPca); % Appling pca to the traj in this sec
        trajSecPointsPca = trajSecPointsPca * pcaTraj; % Oriented traj sec points with its pca

        % Cutting traj until the curvature is small enough, applying the
        % same methot but cutting by step/correctingStep
        correctedStep = step;
        while range(trajSecPointsPca(:,2)) > maxCurvature
            correctedStep    = correctedStep * (maxCurvature / range(trajSecPointsPca(:,2)));
            lastPoint        = find(pdist2(traj.points, traj.points(firstPoint,:)) < correctedStep, 1, 'last');
            idsTrajSec       = (firstPoint:lastPoint)'; 
            trajSecPointsPca = traj.points(idsTrajSec,:) - mean(traj.points(idsTrajSec,:)); 
            pcaTraj          = PcaFlattering(trajSecPointsPca);
            trajSecPointsPca = trajSecPointsPca * pcaTraj;
        end
     end
    sections{j} = idsTrajSec(1:end-1);
    
    firstPoint = lastPoint;
    
end

