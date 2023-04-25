function initialPos = moveRobotsAway(robot, platformMap)
% move the unoccupied robots away from the occupied robot, without concern
% for where to (i.e. just want to isolate the rat to the current platform).

%% need to pick next platforms in a different way, since we want
% them away from the stationary platform, not adjacent to it.
% 3 possibilities: 1) both moving platforms are adjacent to the
% goal, therefore both need to move to the middle ring; 2) only one
% moving platform is adjacent to the goal, and it can move freely
% to the middle ring; 3) only one moving platform is adjacent to
% the goal, but it can't move freely to the middle ring, so the
% second platform also needs to move;

statRobot = find(strcmp({robot(:).movState}, 'stationary'));
nonStatRobots = find(~ismember(1:3, statRobot));

[initialPos, ~] = ...
    getInitPos_CircularPlatforms(robot, platformMap);
adjacent2goal = false(1,2);
[rings, ~] = getRings(robot(statRobot).pos, platformMap);

for r = 1:2
    if ~isempty(intersect(robot(nonStatRobots(r)).pos, rings.inner))
        adjacent2goal(r) = true;
    end
end

if length(find(adjacent2goal)) == 2
    % then both need to move.
    movBoth = true;

else % need to check if both need to move
    adjRobot = find(adjacent2goal);
    nonAdjRobot = find(~ismember([1 2], adjRobot));
    [nonAdjRings, ~] = ...
        getRings(robot(nonStatRobots(nonAdjRobot)).pos, platformMap);

    % if any of the adj robots initial positions are not in the
    % nonAdj robot's inner ring, then only the adj robot needs to
    % move
    innerRingFlag = false(1, length(initialPos{adjRobot}));
    for p = 1:length(initialPos{adjRobot})
        innerRingFlag(p) = ismember(initialPos{adjRobot}(p), ...
            nonAdjRings.inner) || ismember(initialPos{adjRobot}(p), ...
            robot(nonStatRobots(nonAdjRobot)).pos);
    end

    if length(find(innerRingFlag)) < length(initialPos{adjRobot})
        % only adjacent robot needs to move
        movBoth = false;

    else
        % both need to move.
        movBoth = true;
    end
end

if movBoth % Just pick first combination of initial positions that are compatible.
    breakFlag = 0;
    for i1 = 1:length(initialPos{1})
        for i2 = 1:length(initialPos{2})
            [initRings, ~] = ...
                getRings(initialPos{1}(i1), platformMap);
            if initialPos{1}(i1) ~= initialPos{2}(i2) && ...
                    ~ismember(initialPos{2}(i2), initRings.inner)
                initialPos{1} = initialPos{1}(i1);
                initialPos{2} = initialPos{2}(i2);
                breakFlag = 1;
                break
            end
        end
        if breakFlag == 1
            break
        end
    end
else
    initialPos{nonAdjRobot} = robot(nonStatRobots(nonAdjRobot)).pos;
    initialPos{adjRobot} = initialPos{adjRobot}(find(~innerRingFlag, 1));
end