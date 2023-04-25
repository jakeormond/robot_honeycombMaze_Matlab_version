function [robotInput, robotsFinal] = constructRobotPaths(robot, pathsFinal, platformMap)
%%
statRobot = find(strcmp({robot(:).movState}, 'stationary'));
nonStatRobots = find(~ismember(1:3, statRobot));

adjFlag1 = checkAdjacent(robot(statRobot).pos, robot(nonStatRobots(1)).pos, platformMap);
adjFlag2 = checkAdjacent(robot(statRobot).pos, robot(nonStatRobots(2)).pos, platformMap);

if ~adjFlag1 && ~adjFlag2
    maxNEpochs = 2;
else
    maxNEpochs = 3;
end

for r = 1:2
    [robotInput{r}, robotsFinal(r)] = ...
        constructPath(robot(nonStatRobots(r)), pathsFinal{r}, platformMap);  

    if length(robotInput{r}) > 2 && maxNEpochs == 2
        if length(robotInput{r}{1}) < 2
            robotInput{r}(1) = [];
        elseif length(robotInput{r}{2}) < 2
            robotInput{r}(2) = [];
        else
            if mod(length(robotInput{r}{1}), 2) == 0
                robotInput{r}{1} = [robotInput{r}{1}, 0, robotInput{r}{2}(2:end)];
            else
                robotInput{r}{1} = [robotInput{r}{1}, robotInput{r}{2}(2:end)];
            end
            robotInput{r}(2) = [];
        end
    end
end
%%
function [robotInput, robotsFinal] = constructPath(robot, path, platformMap)

currDir = robot.dir;
directions = NaN(length(path), 1);
directions(1) = currDir;
for p = 2:length(path)
    % get direction to next platform
    currPlat = path(p-1);
    nextPlat = path(p);
    directions(p) = getDirection(currPlat, nextPlat, platformMap);
end

robotsFinal.pos = path(end);
robotsFinal.dir = directions(end);

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


% the robots move in 2 to 3 segments. If the robot starts with a
% turn, that will be the first movement. Then it moves up to the
% second last platform of the path, then into the final position.
robotInput = {};
robotInput{1} = 0; % first digit is turn around, only
% necessary with hex platform

if isempty(plats2turns)
    if segmentLengths == 1
        robotInput{1} = [robotInput{1}, 1];
    else
        robotInput{1} = [robotInput{1}, segmentLengths -1];
        robotInput{2} = [0 1];
    end

elseif plats2turns(1) == 1
    robotInput{1} = [robotInput{1}, 0, turnsLines(1)];

    robotInput{2} = 0;
    for s = 1:length(segmentLengths) - 1
        robotInput{2} = [robotInput{2}, ...
            segmentLengths(s), turnsLines(s+1)];
    end

    if segmentLengths(end) > 1
        robotInput{2} = [robotInput{2}, ...
            segmentLengths(end) - 1];
    end

    robotInput{3} = [0 1];
else
    for s = 1:length(segmentLengths) - 1
        robotInput{1} = [robotInput{1}, ...
            segmentLengths(s), turnsLines(s)];
    end

    if segmentLengths(end) > 1
        robotInput{1} = [robotInput{1}, ...
            segmentLengths(end) - 1];
    end

    robotInput{2} = [0 1];
end
if length(robotInput) > 1 && length(robotInput{2}) == 1 
    robotInput(2) = [];
end
%% remove unnecessary input units



