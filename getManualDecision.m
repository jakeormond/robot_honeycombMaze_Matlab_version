function newStatPlat = getManualDecision(robot, nextPlats, spinFlag)
%%
while true
    newStatPlat = input(['what platform did the animal choose (1 = ' ...
        num2str(nextPlats(1)) ', 2 = ' num2str(nextPlats(2)) ')? (press 3 if he won''t move to spin the platform)\n']);
    
    if isempty(newStatPlat) || newStatPlat < 1 || newStatPlat > 3
        continue
        
    elseif newStatPlat == 3 % spin platform
        if spinFlag
            robotCom = './honeycomb_fast 0 0 1';
            degreesTurned = 60;
            spinFlag = false;
        else
            robotCom = './honeycomb_fast 0 0 5';
            degreesTurned = -60;
            spinFlag = true;
        end
        robotCh{statRobot} =  ...
            sshfrommatlabissue_dontwait(robotCh{statRobot}, robotCom);
        
        newDirection = robot(statRobot).dir + degreesTurned;
        if newDirection < 0
            newDirection = newDirection + 360;
        elseif newDirection >= 360
            newDirection = newDirection - 360;
        end
        robot(statRobot).dir = newDirection;
        
        tic
        while true
            elapsedTime = toc;
            if elapsedTime > 1
                break
            end
        end
        
    elseif newStatPlat == 1 || newStatPlat == 2
        break
    end
end
newStatPlat = nextPlats(newStatPlat);