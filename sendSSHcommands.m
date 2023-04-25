function [robot, decChange, newCropNums] = sendSSHcommands(robot, robotInput, nEpochs, ...
    duration, robotCh, atGoal, robotsFinal, oldCropNums, platCoor)
%%
statRobot = find(strcmp({robot.current(:).movState}, 'stationary'));
nonStatRobots = find(~ismember(1:3, statRobot));
newCropNums = oldCropNums;
presentDir = cd;

%%
decChange = [];
cropFlag = false;

for e = 1:nEpochs
    tic
    
    % crop THIS IS INCORRECT IF AN
    if (e > 1 || nEpochs < 3) && ~atGoal && ~cropFlag % CHECK THIS!!!
        allPos = [robot.current(statRobot).pos; ...
            vertcat(robotsFinal(:).pos)];
        newCropNums  = getCropNums(allPos, platCoor);
        
        if ~isequal(sort(newCropNums), sort(oldCropNums))
            cd ../..
            writematrix(newCropNums, 'cropNums.csv')
            cd(presentDir)
        end
        cropFlag = true;    
    end
    
    % send commands    
    for r = 1:2
        if isempty(robotInput{r}{e})
            continue
        end

        robotCom = './honeycomb_fast ';
        robotComVar = robotInput{r}{e};

        for v = 1:length(robotComVar)
            if v < length(robotComVar)
                robotCom = [robotCom num2str(robotComVar(v)) ' '];
            else
                robotCom = [robotCom num2str(robotComVar(v))];
            end
        end

        robotID = nonStatRobots(r);
        robotCh{robotID} =  ...
            sshfrommatlabissue_dontwait(robotCh{robotID}, robotCom);
    end

    duraTemp = max(duration{1}(e), duration{2}(e));
    % duraTemp = 0;
    while true
        elapsedTime = toc;
        if elapsedTime > duraTemp
            break
        end
    end

    if (e == 1 && nEpochs == 3) || (e == 1 && atGoal) % if we are on a turn-only step

        % ONLY IF CHOICE PLATFORMS ARE ADJACENT TO STATIONARY PLATFORM,
        % STILL NEED TO CODE THIS!!!!!!!!!

        decChange = input('did the animal change it''s decision? (''y'', hit enter for no])\n', 's');

        if decChange == 'y' % need to update robot directions and exit for loop

            if atGoal
                atGoal = false;
            end

            % revert to previous movement state
            for r = 1:3
                robot.previous(r).movState = robot.current(r).movState;
                robot.previous(r).pos = robot.current(r).pos;

            end

            for r = 1:2
                if isempty(robotInput{r}{1})
                    continue
                end

                linesTurned = robotInput{r}{1}(3);
                if linesTurned > 3
                    linesTurned = -(6 - linesTurned);
                end
                degreesTurned = linesTurned * 60;
                newDirection = robot.current(nonStatRobots(r)).dir + degreesTurned;
                if newDirection < 0
                    newDirection = newDirection + 360;
                elseif newDirection >= 360
                    newDirection = newDirection - 360;
                end

                robot.previous(nonStatRobots(r)).dir = ...
                    robot.current(nonStatRobots(r)).dir;
                
                robot.current(nonStatRobots(r)).dir = newDirection;
            end
            decChange = true;
            return
        end
%     elseif ~cropFlag       
%         allPos = [robot.current(statRobot).pos; ...
%             vertcat(robotsFinal(:).pos)];
%         newCropNums  = getCropNums(allPos, platCoor);
%         
%         if ~isequal(sort(newCropNums), sort(oldCropNums))
%             cd ../..
%             writematrix(newCropNums, 'cropNums.csv')            
%             cd(presentDir)
%         end
%         cropFlag = true;
    end
end

% update robot
if ~strcmp(decChange, 'y')
    
    % update previous state for all 3 robots
    for r = 1:3
        robot.previous(r).pos = robot.current(r).pos;

        robot.previous(r).dir = robot.current(r).dir;
        
        robot.previous(r).movState = robot.current(r).movState;
    end
    
    % update current state for the 2 moving robots
    for r = 1:2
        robot.current(nonStatRobots(r)).pos = robotsFinal(r).pos;
        robot.current(nonStatRobots(r)).dir = robotsFinal(r).dir;        
    end
end

decChange = false;

    
    
    
    
    
    
    
    
    