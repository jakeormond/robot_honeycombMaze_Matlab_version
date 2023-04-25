function robotInput = constructRobotSimplePaths(robot, initialPos, platformMap)
%%
robotInput = cell(1,2);
nonStatRobots = find(strcmp({robot(:).movState}, 'moving'));

for r = 1:2
    currPlat = robot(nonStatRobots(r)).pos;
    nextPlat = initialPos{r};

    if currPlat == nextPlat
        robotInput{r} = {};
        continue % robot doesn't need to move
    end
    currDir = robot(nonStatRobots(r)).dir;
    direction = getDirection(currPlat, nextPlat, platformMap);

    turn = direction - currDir;

    if turn < 0
        turn = turn + 360;
    end

    turnLines = turn/60;


    % the robots move in 2 to 3 segments. If the robot starts with a
    % turn, that will be the first movement. Then it moves up to the
    % second last platform of the path, then into the final position.
    robotInput{r} = {};
    robotInput{r}{1} = [0, 0, turnLines];
    robotInput{r}{2} = [0, 1];
end