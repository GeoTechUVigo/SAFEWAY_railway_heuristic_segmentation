function [c] = RecordInformation(components, status)
% Function to generate a container with the information in status and the
% number of elements of each type.


% number of each type of element in components
rails=0;
for i=1:length(components.rails)
    rails = rails + length(components.rails{i});
end
% for all track
catenaries = 0;
for i=1:length(components.cables.pairs)
    catenaries = catenaries + length(components.cables.pairs{i});
end
contacts = catenaries;

other_wires = length(components.cables.others);

droppers = 0;
for i=1:length(components.droppers)
    for j=1:length(components.droppers{i})
        droppers = droppers + length(components.droppers{i}{j});
    end
end

masts = length(components.masts);

lights = length(components.signals.light);
marks = length(components.signals.stone);
signs = length(components.signals.big);
trafficLights = length(components.signals.trafficLight);
inMast = length(components.signals.inMast);

% Generate the container
c = containers.Map;
% execution times in status
c("execution times") = status;
c("catenary wires") = catenaries;
c("contact wires") = contacts;
c("droppers") = droppers;
c("lights") = lights;
c("marks") = marks;
c("masts") = masts;
c("other wires") = other_wires;
c("rails") = rails;
c("signs") = signs;
c("signs in masts") = inMast;
c("traffic lights") = trafficLights;

end

