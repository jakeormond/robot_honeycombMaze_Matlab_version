function [robot, turnOnlyFlag] = sendSSHcommands_simple_wDisplay(robot, robotInput, pathsFinal, ...
    duration, robotCh)
%%
statRobot = find(strcmp({robot(:).movState}, 'stationary'));
nonStatRobots = find(~ismember(1:3, statRobot));

%%
tic

turnOnlyFlag = true;
for r = 1:2
    if isempty(robotInput{r})
        continue
    end

    robotCom = './honeycomb_fast ';
    robotComVar = robotInput{r};

    for v = 1:length(robotComVar)
        if v < length(robotComVar)
            robotCom = [robotCom num2str(robotComVar(v)) ' '];
        else
            robotCom = [robotCom num2str(robotComVar(v))];
        end

        if mod(v,2) == 0 && robotComVar(v) ~= 0
            turnOnlyFlag = false;
        end
    end
    
    robotID = nonStatRobots(r);
    robotCh{robotID} =  ...
        sshfrommatlabissue_dontwait(robotCh{robotID}, robotCom);

    % update the robot structure
    % first add up the turns
    turns = 0;
    if length(robotInput{r}) > 2
        indTurns = robotInput{r}(3:2:end);    
    else
        indTurns = 0;
    end
    turns = sum(indTurns);
    turns = mod(turns, 6);
    if turns > 3
        turns = turns - 6;
    end

    robotDirInit = robot(nonStatRobots(r)).dir;
    newDir = robotDirInit + (60 * turns);
    if newDir >= 360
        newDir = newDir - 360;
    elseif newDir < 0
        newDir = newDir + 360;
    end
    robot(nonStatRobots(r)).dir = newDir;


    % then the number of straightlines
    indLines = robotInput{r}(2:2:end);
    lines = sum(indLines);
    if lines > 0
        robotPosInit = robot(nonStatRobots(r)).pos;
        posInd = find(pathsFinal{r} == robotPosInit,1);
        newPos = pathsFinal{r}(posInd + lines);
        robot(nonStatRobots(r)).pos = newPos;
    end

    fprintf('robot%u position = %u, direction = %u\n', nonStatRobots(r), ...
        robot(nonStatRobots(r)).pos, robot(nonStatRobots(r)).dir)

end

duraTemp = max(duration(1), duration(2));
while true
    elapsedTime = toc;
    if elapsedTime > duraTemp
        break
    end
end


    
    
    
    
    
    
    
    
    