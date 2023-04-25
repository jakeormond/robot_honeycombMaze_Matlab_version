function robot = sendSSHcommands_simple_TESTING(robot, robotInput, pathsFinal, ...
    duration, robotCh)
%%
tic

for r = length(robot):-1:1
    robotCom = './honeycomb_fast ';
    robotComVar = robotInput;

    for v = 1:length(robotComVar)
        if v < length(robotComVar)
            robotCom = [robotCom num2str(robotComVar(v)) ' '];
        else
            robotCom = [robotCom num2str(robotComVar(v))];
        end
    end
    
%     robotCh{r} =  ...
%         sshfrommatlabissue_dontwait(robotCh{r}, robotCom);

    % update the robot structure
    % first add up the turns
    turns = 0;
    if length(robotInput) > 2
        indTurns = robotInput(3:2:end);    
    else
        indTurns = 0;
    end
    turns = sum(indTurns);
    turns = mod(turns, 6);
    if turns > 3
        turns = turns - 6;
    end

    robotDirInit = robot(r).dir;
    newDir = robotDirInit + (60 * turns);
    if newDir >= 360
        newDir = newDir - 360;
    elseif newDir < 0
        newDir = newDir + 360;
    end
    robot(r).dir = newDir;


    % then the number of straightlines
    indLines = robotInput(2:2:end);
    lines = sum(indLines);
    if lines > 0
        robotPosInit = robot(r).pos;
        posInd = find(pathsFinal{r} == robotPosInit,1);
        newPos = pathsFinal{r}(posInd + lines);
        robot(r).pos = newPos;
    end
end

while true
    elapsedTime = toc;
    pause(0.1)
    if elapsedTime > duration
        break
    end
end


    
    
    
    
    
    
    
    
    