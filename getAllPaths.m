function paths = getAllPaths(robot, nextPlats, initialPos, platformMap)
%
%%
statRobot = find(strcmp({robot(:).movState}, 'stationary'));
nonStatRobots = find(~ismember(1:3, statRobot));

[rings, ~] = getRings(robot(statRobot).pos, platformMap);
mRing = rings.middle;
lengthInit = max(cellfun(@length, initialPos));
paths = cell(2, 2, lengthInit, 2);
for r = 1:2
    for p = 1:2
        finalPos = nextPlats(p);
        [rings, ~] = getRings(finalPos, platformMap);
        finalAdj = rings.inner;

        stopPos = intersect(finalAdj, mRing); % position in middle ring from which robot moves to final position
        for i = 1:length(initialPos{r})
            startPos = initialPos{r}(i);
            for d = 1:2 % direction around ring
                if d == 1
                    pathLong = [mRing; mRing];
                else
                    pathLong = flipud([mRing; mRing]);
                end

                startInd = find(pathLong == startPos);

                if isempty(startInd) % see getInitPos_CircularPlatforms.m; this only occurs in one specific condition
                    % initial position is in outer ring, so needs to
                    % move to middle ring
                    platRings = getRings(startPos, platformMap);
                    secondPlat = intersect(platRings.inner, pathLong); % one of these will be the original plat position; delete it
                    secondPlat(secondPlat == robot(nonStatRobots(r)).pos) = [];
                    secondInd = find(pathLong == secondPlat);
                    pathLong = [startPos; pathLong(secondInd(1):secondInd(2) - 1)];
                else
                    pathLong = pathLong(startInd(1):startInd(2) - 1);
                end

                [~, stopInd, ~] = intersect(pathLong, stopPos);
                stopInd = min(stopInd);
                pathShort = pathLong(1:stopInd);

                if pathShort(1) == robot(nonStatRobots(r)).pos
                    pathShort(1) = [];
                end

                paths{r, p, i, d} = [pathShort; finalPos];
            end
        end
    end
end