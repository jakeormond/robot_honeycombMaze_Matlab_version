function robotInput = constructRobotPaths_Simple(robot, pathsFinal, platformMap)
%%

robotInput = ...
    constructPath(robot, pathsFinal, platformMap);

%%
function robotInput = constructPath(robot, path, platformMap)

currDir = robot.dir;
directions = NaN(length(path), 1);
directions(1) = currDir;
for p = 2:length(path)
    % get direction to next platform
    currPlat = path(p-1);
    nextPlat = path(p);
    directions(p) = getDirection(currPlat, nextPlat, platformMap);
end

% find turns
directionDiff = diff(directions);
plats2turns = find(directionDiff ~= 0);

segmentLengths = diff([plats2turns; length(path)]);

turns = directionDiff(directionDiff~=0);
turns(turns < 0) = turns(turns < 0) + 360;
turnsLines = turns/60;

if isempty(plats2turns)
    segmentLengths = length(path) - 1;
elseif plats2turns(1) ~= 1
    segmentLengths = [plats2turns(1)-1; segmentLengths];
end

robotInput(1) = 0; % first digit is turn around, only
% necessary with hex platform

if isempty(plats2turns)
    robotInput = [robotInput, segmentLengths];

elseif plats2turns(1) == 1
    robotInput = [robotInput, 0, turnsLines(1)];

    for s = 1:length(segmentLengths) - 1
        robotInput = [robotInput, ...
            segmentLengths(s), turnsLines(s+1)];
    end

    robotInput = [robotInput, ...
        segmentLengths(end)];


else
    for s = 1:length(segmentLengths) - 1
        robotInput = [robotInput, ...
            segmentLengths(s), turnsLines(s)];
    end

    robotInput = [robotInput, ...
        segmentLengths(end)];

end





