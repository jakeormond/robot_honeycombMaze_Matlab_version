% robotMaze_PathSelection_Crop
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
load robotInit.mat
cd(presentDir)

%% connect to robots
% MAKE THIS DEPENDENT ON WHETHER THE CONNECTION IS ALREADY ESTABLISHED

if ~exist('robotCh', 'var')
    robotCh = cell(1,3);
end
robotCh = connect2Robots(robot.current, robotCh);

%% set initial robot positions
for r = 1:3
    robot.current(r).pos = input(['what position for robot ' num2str(r) '?\n']);    % robots 2 and 3 should be placed 1 spot away from this position
end               
robot.previous = robot.current;

goalPlatform = input('what position for goal?');
if isempty(goalPlatform) % then must be in goal folder
    goalDir = pwd;
    goalInd = regexp(goalDir, '\w*goal\w*');
    goalPlatform = str2double(goalDir(goalInd+4:end));
end


%% set data storage
[fileName, choices] = setRobotDataStorage(goalPlatform);
presentDir = cd;
%% get initial crop positions for video tracking

cd ../..
load platformCoordinates.mat
currPlat = robot.current(1).pos;
cropNums  = getCropNums(currPlat, platCoor);
writematrix(cropNums, 'cropNums.csv')
cd(presentDir)

fprintf('\nStart video now. Press any button to continue. \n')
pause


%% set variables

difficulty = 'easy';
% difficulty = 'hard';

atGoal = false;

spinFlag = false; % spin flag, used to get animal moving
%% trial runs within a loop
loopCounter = 1;
decChange = false;
nextPlats = [];

while ~atGoal    
    %% pick new platform positions
    % find previous combinations of platforms, if any, previously chosen
    % from that location.    
    if isfield(choices, 'all')
        choicesTotal = [choices.all, choices.(fileName)];
    else
        choicesTotal = choices.(fileName);
    end
    
    if loopCounter == 1
        currentPlat = robot.current(1).pos;
    else
        currentPlat = choices.(fileName)(loopCounter-1).chosenPlatform; 
    end

    prevVisits = find([choicesTotal(:).startPlatform] == ...
        currentPlat);
    prevVisits(prevVisits >= length(choicesTotal) - 1) = [];
    if ~isempty(prevVisits)
        prevChoices = [vertcat(choicesTotal(prevVisits).chosenPlatform), ...
            vertcat(choicesTotal(prevVisits).unchosenPlatform)];
    else
        prevChoices = [];
    end
    
    % get distance to goal of current platform
    statRobot = find(strcmp({robot.current(:).movState}, 'stationary'));
    currDistance = cartesianDistance(robot.current(statRobot).pos, ...
        goalPlatform, platformMap);
    
    if currDistance == 0 % if animal has reached the goal, just move choice robots away
        atGoal = true;
        initialPos = moveRobotsAway(robot.current, platformMap);
        
    else
        
        if ~decChange || (decChange && ...
                ~strcmp(robot.current(statRobot).movState, ...
                robot.previous(statRobot).movState))
            % pick new platforms and get crop coordinate UNLESS animal has
            % rechosen the same platform (ELSE condition)
            
            prevPlats = nextPlats;
            nextPlats = pickNextPlatforms(prevChoices, robot.current, platformMap, ...
                excludedPlats, goalPlatform, difficulty);
            
        else
            nextPlats = futurePlats;
        end          
    end
    %% get new crop coordinates  CHECK THIS!!!!
%     currPlat = robot.current(1).pos;
%     cropNums  = getCropNums([currPlat; nextPlats], platCoor);
%     cd ../..
%     writematrix(cropNums, 'cropNums.csv')    
%     cd(presentDir)
    
    %%
    if ~atGoal
                
        %% get initial positions
        [initialPos, movFirst] = ...
            getInitPos_CircularPlatforms(robot.current, platformMap);
        if movFirst ~= 0
            movSecond = setdiff([1, 2], movFirst);
        else
            movSecond = 0;
        end
        %% construct paths from initial positions to final positions
        % robots move around the stationary robot's middle ring. Calculate all
        % possible paths for each moving robot to each next platform.
        
        paths = getAllPaths(robot.current, nextPlats, initialPos, platformMap);

        %% check compatibility of all possible pairs of paths
        % The possible combinations are 1) Robot1 to platform1, Robot2 to
        % platform2, and 2) Robot1 to platform2, and Robot2 to platform1.
        % Within each possible combinations, all directions are possible (i.e.
        % directions 1 and 2, so 4 possible combinations). In total, there are
        % 8 possible combinations that need to be checked.
        
        [minLengths, maxLengths] = checkAllPaths(robot.current, initialPos, ...
            paths, movFirst, platformMap);
       
        pathsFinal = pickFinalPaths(robot.current, paths, minLengths, ...
            maxLengths);
        
        %% make figure of current and future platform positions
        % distance between platform rows is 1 row = 1 a.u.
        % distance between columns is = 1 a.u. / tan(30deg) = 1.7321 a.u.
        currAndFutureFigure(robot.current, pathsFinal, platformMap)      
                
        %% construct robot inputs path
        % this consists of the sequence of linear motions (defined by number of gaps to
        % cross) interleaved with turns (defined by number of lines to cross)
        
        [robotInput, robotsFinal] = ...
            constructRobotPaths(robot.current, pathsFinal, platformMap);        
        
    else % construct simple path for robots moving away from goal at trial end
        
        robotInput = constructRobotSimplePaths(robot.current, initialPos, platformMap);

    end
    %% determine timing of various paths

    [duration, robotInput, nEpochs] = getPathTimings(robotInput, timePerGap, timePerLine);

    %% send ssh commands
    
    [robot, decChange, cropNums] = sendSSHcommands(robot, robotInput, nEpochs, ...
        duration, robotCh, atGoal, robotsFinal, cropNums, platCoor);

%     [robot, decChange] = sendSSHcommands_TEST(robot, robotInput, nEpochs, ...
%         duration, robotCh, atGoal, robotsFinal);
    
    %% get result 
    if decChange
        futurePlats = nextPlats;
        nextPlats = prevPlats;
        loopCounter = loopCounter - 1;  
    end

    if ~atGoal || decChange
        
        newStatPlat = getManualDecision(robot.current, nextPlats, spinFlag);
        
        currTime = clock; currTime = currTime(4:6);
        
        choices.(fileName)(loopCounter).time = [num2str(currTime(1)) ':' ...
           num2str(currTime(2)) ':' num2str(round(currTime(3)))];
       
        statRobot = find(strcmp({robot.current(:).movState}, 'stationary'));
        
        if ~decChange % if animal changed its mind, start platform stays the same
            choices.(fileName)(loopCounter).startPlatform = robot.current(statRobot).pos;
        end
        choices.(fileName)(loopCounter).chosenPlatform = newStatPlat;
        choices.(fileName)(loopCounter).unchosenPlatform = nextPlats(nextPlats ~= newStatPlat);

        for r = 1:3
            if robot.current(r).pos == newStatPlat             
                robot.current(r).movState = 'stationary';
            else
                robot.current(r).movState = 'moving';
            end
        end
        
        nonStatRobots = find(~ismember(1:3, statRobot));
        loopCounter = loopCounter + 1;
    end 
    close all  
end

%% tell video tracking that trial is over
writematrix([],'endTrial.txt');

%% 
if isfield(choices, 'all')
    choices.all = [choices.all, choices.(fileName)];
else
    choices.all = choices.(fileName);
end

save(fileName, 'choices')
cd ..
save choices.mat choices 
























