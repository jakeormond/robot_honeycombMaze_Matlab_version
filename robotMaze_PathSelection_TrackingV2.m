% robotMaze_PathSelection_TrackingV2
% requires "SSH From Matlab" from Matlab file exchange; run commmand
% "javaaddpath('ganymed-ssh2-build250.jar')" from SSH From Matlab folder
% This version is from May 2022, and includes code to calculate crop points
% around the active platform positions which will be saved and opened by
% the video tracking program (most likely running in Bonsai). 
%% LOAD INITAL STATE
presentDir = cd;
folderInd = regexp(presentDir, '\') + 1;
if strcmp(presentDir(folderInd(end) : folderInd(end) + 3), 'goal')
    cd ../..
elseif strcmp(presentDir(folderInd(end) : folderInd(end) + 3), datestr(date, 'dd-mm-yyyy'))  
    cd ..
end
load robotInit.mat % this includes platform map, initial directions, timing of movement, and initial robot states
% load robotInit2.mat % UNCOMMENT IF YOU NEED TO USE THE SECOND SET OF
% ROBOTS
cd(presentDir)

%% connect to robots
if ~exist('robotCh', 'var')
    robotCh = cell(1,3);
end
robotCh = connect2robots(robot, robotCh);

%% set initial robot positions and goal position
for r = 1:3
    robot(r).pos = input(['what position for robot ' num2str(r) '?\n']);    % robots 2 and 3 should be placed 1 spot away from this position
end               

goalPlatform = input('what position for goal?');
if isempty(goalPlatform) % then must be in goal folder
    goalDir = pwd;
    goalInd = regexp(goalDir, '\w*goal\w*');
    goalPlatform = str2double(goalDir(goalInd+4:end));
end


%% set data storage
[fileName, choices] = setRobotDataStorage(goalPlatform);
presentDir = cd;

%% save filenames for Bonsai to use and set Bonsai path
if ~exist('bonsaiPath', 'var')
    bonsaiPath = uigetdir(presentDir, 'select video folder');
end

cd(bonsaiPath)

writematrix([fileName '.avi'],'video_FileName.csv')
writematrix([fileName 'B.avi'],'video_FileName2.csv')
writematrix(['videoTS_' fileName '.csv'],'videoTS_FileName.csv')
writematrix(['cropTimes_' fileName '.csv'],'cropTimes_FileName.csv')
writematrix(['cropValues_' fileName '.csv'],'cropValues_FileName.csv')
writematrix(['pulseTS_' fileName '.csv'],'pulseTS_FileName.csv')

cd(presentDir)

%% get initial crop positions for video tracking

cd ../..
% load platformCoordinates.mat
% load platCoorUpdate_July17.mat
load platformCoordinates_Auto.mat
platCoor = platformCoordinates_Auto; clear platformCoordinates_Auto 
currPlat = robot(1).pos;
cropNums  = getCropNums(currPlat, platCoor);
writematrix(cropNums, 'cropNums.csv')
cd(presentDir)

%% start video and ephys

fprintf('\nStart video and ephys now. Press any button to continue. \n')
pause
%% delete filenames for Bonsai, since they have now been read in by Bonsai

cd(bonsaiPath)
delete('video_FileName.csv')
delete('videoTS_FileName.csv')
delete('cropTimes_FileName.csv')
delete('cropValues_FileName.csv')
delete('pulseTS_FileName.csv')

cd(presentDir)

%% set variables

% difficulty = 'easy'; 
difficulty = 'hard';

atGoal = false;

spinFlag = false; % spin flag, used to get animal moving

loopCounter = 1;
loopCounter2 = 1;

decChange = false; % decision change

prevPlats = [];
nextPlats = []; % next choice platforms
futurePlats = []; % next choices if he changes his mind

choicesFinal = [];

%% trial runs within a loop

while ~atGoal    
    %% pick next platforms 

    % get moving and stationary robots
    statRobot = find(strcmp({robot(:).movState}, 'stationary'));
    movingRobots = find(strcmp({robot(:).movState}, 'moving'));

    % get list of previous choices
    if isfield(choices, 'all')
        choicesTotal = [choices.all, choices.(fileName)];
    else
        choicesTotal = choices.(fileName);
    end
    
    % find choices previously made from the same platform
    prevVisits = find([choicesTotal(:).startPlatform] == ...  
        robot(statRobot).pos);
    
    if ~isempty(prevVisits)
        prevChoices = [vertcat(choicesTotal(prevVisits).chosenPlatform), ...
            vertcat(choicesTotal(prevVisits).unchosenPlatform)];
    else
        prevChoices = [];
    end

    % get distance to goal of current platform    
    currDistance = cartesianDistance(robot(statRobot).pos, ...
        goalPlatform, platformMap);
    if currDistance == 0
        atGoal = true;
        initialPos = moveRobotsAway(robot, platformMap);
    end

    
    % next platforms
    if ~isempty(futurePlats) % i.e. rat has changed his mind twice, so use previous set of nextPlats
        futurePlatsTemp = nextPlats;
        nextPlats = futurePlats;
        futurePlats = futurePlatsTemp; clear futurePlatsTemp

    else
        if decChange % first decision change, so save futurePlats
            futurePlats = nextPlats;
        end

        if atGoal % if animal has reached the goal, just move choice robots away
            nextPlats = [0, 0];            

        else
            nextPlats = pickNextPlatforms(prevChoices, robot, platformMap, ... % pick the new ones
                excludedPlats, goalPlatform, difficulty);
        end
    end
              
    %% MAIN LOOP
    if ~atGoal
           
        %% get initial positions
        [initialPos, movFirst] = ...
            getInitPos_CircularPlatforms(robot, platformMap);

        if movFirst ~= 0 % check if one robot needs to move before the other
            movSecond = setdiff([1, 2], movFirst); % CHECK IF THIS IS NECESSARY
        else
            movSecond = 0;
        end
        %% construct paths from initial positions to final positions
        % robots move around the stationary robot's middle ring. Calculate all
        % possible paths for each moving robot to each next platform.
        
        paths = getAllPaths(robot, nextPlats, initialPos, platformMap);

        %% check compatibility of all possible pairs of paths
        % The possible combinations are 1) Robot1 to platform1, Robot2 to
        % platform2, and 2) Robot1 to platform2, and Robot2 to platform1.
        % Within each possible combinations, all directions are possible (i.e.
        % directions 1 and 2, so 4 possible combinations). In total, there are
        % 8 possible combinations that need to be checked.
        
        [minLengths, maxLengths] = checkAllPaths(robot, initialPos, ...
            paths, movFirst, platformMap);

        %% PICK THE FINAL PATHS
       
        pathsFinal = pickFinalPaths(robot, paths, minLengths, ...
            maxLengths);
        
        %% make figure of current and future platform positions
        % distance between platform rows is 1 row = 1 a.u.
        % distance between columns is = 1 a.u. / tan(30deg) = 1.7321 a.u.
        close all
        currAndFutureFigure(robot, pathsFinal, platformMap)      
                
        %% construct robot inputs path
        % this consists of the sequence of linear motions (defined by number of gaps to
        % cross) interleaved with turns (defined by number of lines to cross)
        
        [robotInput, robotsFinal] = ...
            constructRobotPaths(robot, pathsFinal, platformMap);        
        
    else % AT THE GOAL, so construct simple paths for robots to move away from the goal 
         for r = 1:2
             if initialPos{r} == robot(movingRobots(r)).pos        
                 pathsFinal{r} = initialPos{r};
             else
                 pathsFinal{r} = [robot(movingRobots(r)).pos; initialPos{r}];
             end
         end

        robotInput = constructRobotSimplePaths(robot, initialPos, platformMap);

    end
    
    %% determine timing of various paths
    % note that this pads the inputs so that both robots get the same
    % number of commands

    [duration, robotInput, nEpochs] = getPathTimings(robotInput, timePerGap, timePerLine);

    %% send ssh commands
    % if the first step is turn only, send the command

    currPlat = robot(statRobot).pos;
    for e = 1:nEpochs
        if e == nEpochs && ~atGoal % if the robots are moving to their final position, we need to crop the video             
             cropNums  = getCropNums([currPlat; nextPlats], platCoor);
             cd ../..
             writematrix(cropNums, 'cropNums.csv')
             cd(presentDir)
        end

        robotCh = connect2robots(robot, robotCh);
        [robot, turnOnlyFlag] = sendSSHcommands_simple(robot, ...
            [robotInput{1}(e), robotInput{2}(e)], ...
            pathsFinal, [duration{1}(e), duration{2}(e)], robotCh);

        % if that was a turn only step, we need to ask if the animal moved
        if turnOnlyFlag && (checkAdjacent(robot(statRobot).pos, ...
                robot(movingRobots(1)).pos, platformMap) ...
                || checkAdjacent(robot(statRobot).pos, ...
                robot(movingRobots(2)).pos, platformMap))
            
            % decChange = input('did the animal change it''s decision? (''y'', hit enter for no])\n', 's');
            
            % OR

            prevStartPlat = choicesFinal(loopCounter2 - 1).startPlatform;
            prevChoicePlats = [choicesFinal(loopCounter2 - 1).chosenPlatform; ...
                choicesFinal(loopCounter2 - 1).unchosenPlatform];

            minPlatTime = 1;
            platform = getDecision_WaitBar(prevStartPlat, prevChoicePlats, ...
                platCoor, cropNums, minPlatTime); % currentPlat 
            if ~isempty(platform)
                if platform ~= currPlat
                    decChange = 'y';
                else
                    decChange = 'n';
                end
            else
                decChange = input('did the animal change it''s decision? (''y'', hit enter for no])\n', 's');
            end


            if decChange == 'y'
               
                % decision changed so swap moving and stationary
                robot(statRobot).movState = 'moving';
                % previously unchosen platform becomes stat robot
                statPlat = choices.(fileName)(loopCounter-1).unchosenPlatform;
                robot([robot(:).pos] == statPlat).movState = 'stationary';

                % update decision
                currTime = clock; currTime = currTime(4:6);
                choices.(fileName)(loopCounter).time = [num2str(currTime(1)) ':' ...
                    num2str(currTime(2)) ':' num2str(round(currTime(3)))];
                choices.(fileName)(loopCounter).startPlatform = ...
                    choices.(fileName)(loopCounter-1).startPlatform;
                choices.(fileName)(loopCounter).chosenPlatform = ...
                    choices.(fileName)(loopCounter-1).unchosenPlatform;
                choices.(fileName)(loopCounter).unchosenPlatform = ...
                    choices.(fileName)(loopCounter-1).chosenPlatform;

                loopCounter = loopCounter + 1;

                if atGoal
                    atGoal = false;
                end

                break
            end

        else
            decChange = [];

            if e == nEpochs && ~atGoal % need to get animal's decision
                % newStatPlat = getManualDecision(robot, nextPlats, spinFlag);
                % OR
                % minPlatTime = 4;
                minPlatTime = 2;
                newStatPlat = getDecision_WaitBar(currPlat, nextPlats, platCoor, cropNums, minPlatTime); % currentPlat
                if isempty(newStatPlat)
                    newStatPlat = getManualDecision(robot, nextPlats, spinFlag);
                end

                robot([robot(:).pos] == newStatPlat).movState = 'stationary';
                robot(~ismember([robot(:).pos], nextPlats)).movState = 'moving';

                prevPlats = nextPlats;
                futurePlats = [];

                % record decision
                currTime = clock; currTime = currTime(4:6);
                choices.(fileName)(loopCounter).time = [num2str(currTime(1)) ':' ...
                    num2str(currTime(2)) ':' num2str(round(currTime(3)))];
                choices.(fileName)(loopCounter).startPlatform = currPlat;
                choices.(fileName)(loopCounter).chosenPlatform = newStatPlat;
                choices.(fileName)(loopCounter).unchosenPlatform = nextPlats(nextPlats ~= newStatPlat);

                loopCounter = loopCounter + 1;

                choicesFinal(loopCounter2).startPlatform = currPlat;
                choicesFinal(loopCounter2).chosenPlatform = newStatPlat;
                choicesFinal(loopCounter2).unchosenPlatform = nextPlats(nextPlats ~= newStatPlat);

                loopCounter2 = loopCounter2 + 1;
              
            end
        end
    end
end
close all
%% tell video tracking that trial is over
cd ..\..
writematrix([],'endTrial.txt');
cd(presentDir)

%% 
if isfield(choices, 'all')
    choices.all = [choices.all, choices.(fileName)];
else
    choices.all = choices.(fileName);
end

save(fileName, 'choices')
cd ..
save choices.mat choices 
cd(presentDir)

%% clear variables
clearvars -except robotCh bonsaiPath

%%
function robotCh = connect2robots(robot, robotCh)

if isempty(robotCh{1}) || isempty(robotCh{2}) || isempty(robotCh{3})% unless already connected
    robotCh = connect2Robots(robot, robotCh);
end

end
























