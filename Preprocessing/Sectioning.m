function [cloud, trajCloud, sections, idxCloud, idxTraj] = Sectioning(cloud, traj)
% Function to split the cloud and traj in sections by length and curvature.
% 
% First, the part of the trajectory corresponding to this cloud is selected
% by timeStamp and bounding box. Besides, the closest points in cloud of
% the first and last point in the selected trajectory are calculated (start
% and last), and the selected trajectory (trajCloud) goes from the closest 
% point to start to the closest point to end. The reasson of that is
% because of the width of the cloud might be points of the trajectory in
% the bounding box but that they are not from this cloud.
%
% All trajCloud is analyzed in a loop for. Points from the last point of 
% the last section to the first point of the rest of trajCloud at step 
% distance forms the next section. That points are oriented. If its
% curvature is higher than the maximum admited, the trajSec is shortened.
%
% Sections can have an overlap between them defined in meter, insteed of
% starting in the next point of the last point of its previous.
% 
% Onces the trajSec has been selected, the cloud of this trajSec is
% calculated. CloudSec is the part of the cloud inside trajSec.timeStamp
% and bounding box + overlap with the next section. To calculate the
% bounding box, the cloud is oriented as trajSec has been oriented. 
% The X limits of the bound box are the dimensions of trajSec, the Y are 
% the same + width (defined), and Z has no limit.
%
% If it is the last part of the cloud, the process is the same but starting
% from the end of the trajectory instead from the last point of the last
% section.
%
% Points far away from all trajSec (more than width) are not in any
% section. Cloud formed by all section is calculated. Same with the 
% trajecroty. Indexes of each section were referred to the input cloud. 
% They are recalculated referring to the new cloud. Same with the 
% trajecroty. The indexes necessary to undo this transformation is idxCloud
% and idxTraj, taht are the points of  output cloud/trajecroty in input 
% cloud/trajecroty.
%
% -------------------------------------------------------------------------
% INPUTS:
%
% cloud : pointCloud_
%   
% traj : trajectory
% -------------------------------------------------------------------------
% OUTPUTS:
%
% cloud : pointCloud_. Cloud with all the points in sections{1:N}.cloud
%
% trajCloud : trajectory. Cloud with all the points in sections{1:N}.traj
%
% sections : cell array with indexes in cloud output (not in cloud input)
%            and trajCloud.
%            section{i}.cloud has the indexes in cloud output in section i
%            section{i}.traj has the indexes in trajCloud in section i
%
% idxCloud : Nx1 numeric. Indexes of cloud output in cloud input.
%
% idxTraj : Nx1 numeric. Indexes of trajCloud in traj.
% -------------------------------------------------------------------------
% Daniel Lamas Novoa.
% Enxeñaría dos materiais, mecánica aplicada e construción.
% Escola de enxeñería industrial
% Grupo de xeotecnoloxía aplicada.
% Universidade de Vigo.
% 28/12/2020

%% Parameters for sections
step         = 100; % max length of each sec
maxCurvature = 5; % max curvature of each section in meters. It is the max range in y
width        = 20;
overlap      = 5; % overlap between sections

sections = [];

%% Trajectory
trajCloud = select(traj, traj.timeStamp >= min(cloud.timeStamp) & traj.timeStamp < max(cloud.timeStamp)); % Section of the trajectory of this cloud by time Stamp
trajCloud = select(trajCloud, all(trajCloud.points >= min(cloud.Location),2) & all(trajCloud.points < max(cloud.Location),2)); % and location bounding box

% Closer to the limits of the cloud
[~, start] = min(pdist2(trajCloud.points(1,:), cloud.Location)); % point in cloud closer to the first traj point
[~, start] = min(pdist2(cloud.Location(start,:), trajCloud.points)); % first point of the traj in cloud
[~, last]  = min(pdist2(trajCloud.points(end,:), cloud.Location)); % same but last point
[~, last]  = min(pdist2(cloud.Location(last,:), trajCloud.points));

trajCloud = select(trajCloud, start:last);

% figure; pcshow(cloud.Location, 'b')
% hold on; pcshow(cloud.Location(a,:), 'g', 'MarkerSize', 100)
% hold on; pcshow(trajCloud.points, 'r', 'MarkerSize',100)

firstPoint      = 1;
restPointsTraj  = true(length(trajCloud.points),1);
restPointsCloud = true(length(cloud.Location),1);

%% Sections
allTrajDone = true;
j = 0;
while allTrajDone
    j = j + 1;

    % it works because all points are sorted by timeStamp
    lastPoint = find(pdist2(trajCloud.points, trajCloud.points(firstPoint,:)) < step & restPointsTraj, 1, 'last'); % Finding the first point of the rest of the traj at step distance.     
    
    if ~isempty(lastPoint) && lastPoint ~= length(trajCloud.points)

        idsTrajSec       = (firstPoint:lastPoint)'; % ids of the traj between first and last point of this section
        trajSecPointsPca = trajCloud.points(idsTrajSec,:) - mean (trajCloud.points(idsTrajSec,:)); % traj sec points centered
        pcaTraj          = PcaFlattering(trajSecPointsPca); % Appling pca to the traj in this sec
        trajSecPointsPca = trajSecPointsPca * pcaTraj; % Oriented traj sec points with its pca

        % Cutting traj until the curvature is small enough, applying the
        % same methot but cutting by step/correctingStep
        correctedStep = step;
        while range(trajSecPointsPca(:,2)) > maxCurvature
            correctedStep    = correctedStep * (maxCurvature / range(trajSecPointsPca(:,2)));
            lastPoint        = find(pdist2(trajCloud.points, trajCloud.points(firstPoint,:)) < correctedStep & restPointsTraj, 1, 'last');
            idsTrajSec       = (firstPoint:lastPoint)'; 
            trajSecPointsPca = trajCloud.points(idsTrajSec,:) - mean(trajCloud.points(idsTrajSec,:)); 
            pcaTraj          = PcaFlattering(trajSecPointsPca);
            trajSecPointsPca = trajSecPointsPca * pcaTraj;
        end

    else % last section
        % if it is empty it will be the last point of traj, so I select the
        % section not looking for the last point, but the first, to have a
        % section with the same length than the others    

        lastPoint        = length(trajCloud.points);
        firstPoint       = find(pdist2(trajCloud.points, trajCloud.points(lastPoint,:)) < step, 1, 'first');
        idsTrajSec       = (firstPoint:lastPoint)'; % ids of the traj between first and last point of this section
        trajSecPointsPca = trajCloud.points(idsTrajSec,:) - mean(trajCloud.points(idsTrajSec,:)); % traj sec points centered
        pcaTraj          = PcaFlattering(trajSecPointsPca); % Appling pca to the traj in this sec
        trajSecPointsPca = trajSecPointsPca * pcaTraj; % Oriented traj sec points with its pca

        % Cutting traj until the curvature is small enough, applying the
        % same methot but cutting by step/correctingStep
        correctedStep = step;
        while range(trajSecPointsPca(:,2)) > maxCurvature
            correctedStep    = correctedStep * (maxCurvature / range(trajSecPointsPca(:,2)));
            firstPoint       = find(pdist2(trajCloud.points, trajCloud.points(lastPoint,:)) < correctedStep, 1, 'first');
            idsTrajSec       = (firstPoint:lastPoint)'; 
            trajSecPointsPca = trajCloud.points(idsTrajSec,:) - mean(trajCloud.points(idsTrajSec,:)); 
            pcaTraj          = PcaFlattering(trajSecPointsPca); % Coordinates centred
            trajSecPointsPca = trajSecPointsPca * pcaTraj; % Coordinates in the new axes   
        end
    end

    % Checking if trajPoinjt is in the the correct order
    if trajSecPointsPca(end,1) < trajSecPointsPca(1,1)
        pcaTraj = RotateAxes(pcaTraj, 180, pcaTraj(:,3)); % Rotatin the axes
        trajSecPointsPca = trajCloud.points(idsTrajSec,:) - mean(trajCloud.points(idsTrajSec,:)); % Coordinates centred
        trajSecPointsPca = trajSecPointsPca * pcaTraj; % Coordinates in the new axes
    end

    %% Cloud

    if lastPoint ~= length(trajCloud.points) % if it is not the end of the cloud

        % Chossing the ids in the rest of the cloud
        % preCloudLocation has the location of the rest of the cloud (cloud not analyzed) with a
        % timeStamp lower than the max + range of the traj. It is to not apply
        % pca to all the cloud
        idsCloudSec = find(cloud.timeStamp <= (max(trajCloud.timeStamp(idsTrajSec)) + range(trajCloud.timeStamp(idsTrajSec))) & restPointsCloud);

        % Applying the same operations that have been applyed to the traj
        % of this sec
        cloudSecLocationPca = cloud.Location(idsCloudSec,:) - mean(trajCloud.points(idsTrajSec,:));
        cloudSecLocationPca = cloudSecLocationPca * pcaTraj;


        % Save points to remove all that are inside the range of traj in X
        idsCloudRemove = idsCloudSec(cloudSecLocationPca(:,1) < (max(trajSecPointsPca(:,1))));
        % Only points close to traj in Y and with less X than the
        % maximum of the traj, considering overlap
        idsCloudSec = idsCloudSec(cloudSecLocationPca(:,1) < (max(trajSecPointsPca(:,1)) + overlap) & abs(cloudSecLocationPca(:,2)) < width/2);

        % Saving ids
        sections{j}.cloud = idsCloudSec; 
        sections{j}.traj  = idsTrajSec(1:end-1); % the last point is for the next section

        % Updating points for the next loop
        restPointsTraj(idsTrajSec(1:end-1)) = false;  % the last point is for the next section
        restPointsCloud(idsCloudRemove)     = false;
        firstPoint                          = lastPoint;

    else % if it is the end of the cloud

        % points with a higher timeStamp thatn the minimum timeStamp of
        % the traj of the sec - its range. They are the points of the
        % end of the cloud
        idsCloudSec = find(cloud.timeStamp >= (min(trajCloud.timeStamp(idsTrajSec)) - range(trajCloud.timeStamp(idsTrajSec))));

        % Applying the same operations that have been applyed to the traj
        % of this sec
        cloudSecLocationPca = cloud.Location(idsCloudSec,:) - mean(trajCloud.points(idsTrajSec,:));
        cloudSecLocationPca = cloudSecLocationPca * pcaTraj;

        % Only points close to traj in Y and and with higher X than the
        % mimimum of the traj
        idsCloudSec = idsCloudSec(cloudSecLocationPca(:,1) > (min(trajSecPointsPca(:,1))) & abs(cloudSecLocationPca(:,2)) < width/2);

        % Saving ids
        sections{j}.cloud = idsCloudSec; 
        sections{j}.traj  = idsTrajSec;

        allTrajDone = false; % end of the while 

    end

%     figure; pcshow(cloud.Location, 'w')
%     hold on; pcshow(trajCloud.points(sections{j}.traj,:), 'y', 'MarkerSize',100)
%     hold on; pcshow(cloud.Location(sections{j}.cloud,:), 'g')
% 
%     figure; pcshow(cloudSecLocationPca, 'w')
%     hold on; pcshow(trajSecPointsPca, 'r', 'MarkerSize',100)
%     hold on; pcshow(cloudSecLocationPca(cloudSecLocationPca(:,1) < (max(trajSecPointsPca(:,1)) + overlap) & abs(cloudSecLocationPca(:,2)) < width/2,:), 'g')

end

%% Selecting only the idx of the cloud that are in any section
idxCloud = [];
for k = 1:length(sections)
    idxCloud = [idxCloud; sections{k}.cloud]; % All idx in sections
end

[idxCloud,~,~] = unique(idxCloud); % Deleting repeated idx
[~,order] = sort(cloud.timeStamp(idxCloud), 'ascend'); % sorting by timeStamp
idxCloud       = idxCloud(order);

cloud = select(cloud, idxCloud); % Selecting only the idx in the cloud 

% Updating the idx of the section in the cutted cloud
for k = 1:length(sections)
    sections{k}.cloud = find(ismember(idxCloud,sections{k}.cloud));
end

% Same process in the traj
idxTraj = find(all(trajCloud.points >= min(cloud.Location),2) & all(trajCloud.points < max(cloud.Location),2));
trajCloud = select(trajCloud, idxTraj); % and location

for k = 1:length(sections)
    sections{k}.traj = find(ismember(idxTraj,sections{k}.traj));
end
end


% figure; pcshow(cloud.Location, [0.5,0.5,0.5]);
% hold on; pcshow(a.points, 'w', 'MarkerSize',100)
% 
% hold on; pcshow(cloud.Location(sections{1}.cloud,:), 'y');
% hold on; pcshow(trajCloud.points(sections{1}.traj,:), 'g', 'MarkerSize',100)
% 
% hold on; pcshow(cloud.Location(sections{2}.cloud,:), 'r');
% hold on; pcshow(trajCloud.points(sections{2}.traj,:), 'b', 'MarkerSize',100)

