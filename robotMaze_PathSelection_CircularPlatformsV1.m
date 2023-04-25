% robotMaze_PathSelection_CircularPlatforms
% requires "SSH From Matlab" from Matlab file exchange; run commmand
% "javaaddpath('ganymed-ssh2-build250.jar')" from SSH From Matlab folder
%% connect to robots
% javaaddpath('ganymed-ssh2-build250.jar')
% robotCh{1}  =  sshfrommatlab('root', '192.168.0.101','');
% robotCh{2}  =  sshfrommatlab('root', '192.168.0.103','');
% robotCh{3}  =  sshfrommatlab('root', '192.168.0.106','');

%% set initial robot positions and directions
% 6 possible directions = N, NE, SE, S, SW, SE
direction.N = 0; direction.NE = 60; direction.SE = 120; directionS = 180;
direction.SW = 240; direction.NW = 300;

% robot initial positions and orientations
robot(1).pos = 158;
robot(1).dir = 300;
robot(1).ip = [192, 168 ,0, 101];

robot(2).pos = 177;
robot(2).dir = 60;
robot(2).ip = [192, 168 ,0, 103];

robot(3).pos = 149;
robot(3).dir = 180;
robot(3).ip = [192, 168 ,0, 106];


statRobot = 1; % stationary robot, other robots are to move to adjacent positions.
nonStatRobots = find(~ismember(1:3, statRobot));

goalPlatform = 170;
atGoal = false;

%% make the platform map
nRows = 28; nCols = [9, 10];
platformMap = makePlatMap(nRows, nCols);
%% make list of platform positions that can't be used for final positions.
% These positions are excluded because it's impossible to navigate robots
% close to walls
excludedPlats{1} = platformMap(1:4, :);
excludedPlats{2} = platformMap(:, 1:2);
excludedPlats{3} = platformMap(end-3:end, :);
excludedPlats{4} = platformMap(:, end-1:end);

for i = 1:4
    excludedPlats{i} = excludedPlats{i}(:);
end
excludedPlats = unique(vertcat(excludedPlats{:}));
excludedPlats(isnan(excludedPlats)) = [];


%% trial runs within a loop
choices(1).currentPlatform = robot(statRobot).pos;
choices(1).choicePlatforms = [0, 0];

choiceCounter = 0;
while ~atGoal
    %% pick new platform positions
    % find previous combinations of platforms, if any, previously chosen
    % from that location.
    % NOTE: SHOULD IDENTIFY HERE IF ALL POSSIBLE CHOICES HAVE BEEN GIVEN,
    % IN WHICH CASE, CHOICES WILL HAVE TO BE REPEATED.....NOT CODED YET!
    prevVisits = find([choices(:).currentPlatform] == robot(statRobot).pos);
    if ~isempty(prevVisits)
        prevChoices = vertcat(choices(prevVisits).choicePlatforms);
    else
        prevChoices = [];
    end
    
    % get distance to goal of current platform
    currDistance = cartesianDistance(robot(statRobot).pos, ...
        goalPlatform, platformMap);
    
    while true
        nextPlats = getNextPlats(robot(statRobot).pos, ...
            [robot(~ismember(1:3,statRobot)).pos], platformMap, excludedPlats);
        % nextPlats = [90, 100];
        nextPlats = sort(nextPlats);
        
        if ~isempty(prevChoices)
            if ~isempty(find(prevChoices(:,1) == nextPlats(1) && ...
                    prevChoices(:,2) == nextPlats(2), 1))
                continue
            end
        end
        
        % calculate distance of all platforms to the goal. At least one of
        % the offered platforms need to be closer to the goal than the
        % animal's current location.
        dist1 = cartesianDistance(nextPlats(1), ...
            goalPlatform, platformMap);
        dist2 = cartesianDistance(nextPlats(2), ...
            goalPlatform, platformMap);
        
        if dist1 < currDistance || dist2 < currDistance
            break
        end
    end
    choiceCounter = choiceCounter+1;
    choices(choiceCounter).choicePlatforms = nextPlats;
    
    %% get initial positions
    [initialPos, movFirst] = ...
        getInitPos_CircularPlatforms(robot(nonStatRobots), ...
        robot(statRobot), platformMap);
    if movFirst ~= 0
        movSecond = setdiff([1, 2], movFirst);
    else
        movSecond = 0;
    end
    %% construct paths from initial positions to final positions
    % robots move around the stationary robot's middle ring. Calculate all
    % possible paths for each moving robot to each next platform.
    
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
                    
                    paths{r, p, i, d} = pathShort;
                end
            end
        end
    end
    
    %% check compatibility of all possible pairs of paths
    % The possible combinations are 1) Robot1 to platform1, Robot2 to
    % platform2, and 2) Robot1 to platform2, and Robot2 to platform1.
    % Within each possible combinations, all directions are possible (i.e.
    % directions 1 and 2, so 4 possible combinations). In total, there are
    % 8 possible combinations that need to be checked.
    
    compMatrix = false(2,length(initialPos{1}),2,length(initialPos{2}),2);
    % staggerMatrix = zeros(2,3,2,3,2);
    maxLengths = NaN(2,length(initialPos{1}),2,length(initialPos{2}),2);
    minLengths = NaN(2,length(initialPos{1}),2,length(initialPos{2}),2);
    pathsTemp = cell(1,2);
    pathsFinal = cell(1,2);
    
    for p1 = 1:2 % robot1's platform, which determines robot2's platform
        p2 = find(~ismember([1 2], p1));
        
        for i1 = 1:length(initialPos{1}) % robot1 start platform
            for d1 = 1:2 % robot1's direction
                pathsTemp{1} = paths{1, p1, i1, d1};
                                
                for i2 = 1:length(initialPos{2})
                    for d2 = 1:2 % robot2's direction
                        
                        if movFirst > 0
                            if d2 == d1 
                                continue % robots must move in opposite directions in this specific condition
                            end
                        end
                        
                        pathsTemp{2} = paths{2, p2, i2, d2};                       
                        
                        compFlag = checkPathCompatibilityV2(pathsTemp, ...
                            d1, d2);
                        
                        compMatrix(p1, i1, d1, i2, d2) = compFlag;
                        % staggerMatrix(p1, i1, d1, i2, d2) = stagger;
                        
                        if ~compFlag
                            continue
                        end
                        
                        maxLengths(p1, i1, d1, i2, d2) = max(length(pathsTemp{1}), ...
                            length(pathsTemp{2}));
                        minLengths(p1, i1, d1, i2, d2) = min(length(pathsTemp{1}), ...
                            length(pathsTemp{2}));
                        
                    
                    end
                end
            end
        end
    end
        
    minMaxLength = min(maxLengths(:));
    minInd = find(maxLengths(:) == minMaxLength);
    [p1, i1, d1, i2, d2] = ind2sub(size(maxLengths), minInd);
    
    if length(p1) > 1
        minLength = NaN(length(p1), 1);
        for m = 1:length(p1)
            minLength(m) = minLengths(p1(m), i1(m), d1(m), i2(m), d2(m));
        end
        [~, minInd] = min(minLength);
    end
        
    p1 = p1(minInd);
    p2 = find(~ismember([1 2], p1));
  
    pathsFinal{1} = [robot(nonStatRobots(1)).pos; ...
        paths{1, p1, i1(minInd), d1(minInd)}; nextPlats(p1)];
    pathsFinal{2} = [robot(nonStatRobots(2)).pos; ...
        paths{2, p2, i2(minInd), d2(minInd)}; nextPlats(p2)];
    
    %% make figure of current and future platform positions
    % distance between platform rows is 1 row = 1 a.u.
    % distance between columns is = 1 a.u. / tan(30deg) = 1.7321 a.u.
    htitle = 'platform locations';
    h=figure('Name', htitle,'NumberTitle','off','Units',...
        'pixels', 'Position', [600 300 1000 500], 'Color', [1 1 1], 'Visible', 'on');
    
    theta = 0:60:360;
    x = cosd(theta);
    y = sind(theta);
    
    for i = 1:2
        inRow = NaN(3,1);
        inCol = NaN(3,1);
        plat = NaN(3,1);
        ipAddr = NaN(3,1);
        for r = 1:3
            if i == 1 || r == statRobot
                currPlat = robot(r).pos;
            else
                movInd = find(nonStatRobots == r);
                currPlat = pathsFinal{movInd}(end);
            end
            inInd = find(platformMap == currPlat);
            [inRow(r), inCol(r)] = ind2sub(size(platformMap), inInd);
            plat(r) = currPlat;
            ipAddr(r) = robot(r).ip(end);
        end
        
        subplot(1,2,i)
        
        for r = 1:3
            if r == statRobot
                pColour = 'k';
            else
                pColour = 'b';
            end
            relRow = inRow(r) - min(inRow);
            relCol = inCol(r) - min(inCol);
            
            plot(x + relCol*1.7321, y + relRow, [pColour '-'], 'LineWidth', 3);
            hold on
            
            txt1 = num2str(ipAddr(r));
            text(relCol*1.7321 - .3, relRow - .4, txt1, 'FontSize', 20)
            txt2 = num2str(plat(r));
            text(relCol*1.7321 - .2, relRow + .1, txt2, 'FontSize', 24)
        end
        xlim([-1 5])
        ylim([-1 5])
        box off
        axis off
        set(gca, 'YDir','reverse')
        
        if i == 1
            title('initial positions')
        else
            title('new positions')
        end
        axis equal
    end
    
    %% construct robot inputs path
    % this consists of the sequence of linear motions (defined by number of gaps to
    % cross) interleaved with turns (defined by number of lines to cross)    
    
    for r = 1:2
        currDir = robot(nonStatRobots(r)).dir;
        directions = NaN(length(pathsFinal{r}), 1);
        directions(1) = currDir;
        for p = 2:length(pathsFinal{r})
            % get direction to next platform
            currPlat = pathsFinal{r}(p-1);
            nextPlat = pathsFinal{r}(p);
            directions(p) = getDirection(currPlat, nextPlat, platformMap);
        end
        
        robotsFinal(r).pos = pathsFinal{r}(end);
        robotsFinal(r).dir = directions(end);        
             
        % find turns
        directionDiff = diff(directions);
        plats2turns = find(directionDiff ~= 0);
        nSegments = length(plats2turns);  
           
        segmentLengths{r} = diff([plats2turns; length(pathsFinal{r})]);

        turns = directionDiff(directionDiff~=0);       
        turns(turns < 0) = turns(turns < 0) + 360;
        turnsLines{r} = turns/60;
        
        if plats2turns(1) ~= 1
            nSegments = nSegments + 1;
            segmentLengths{r} = [plats2turns(1)-1; segmentLengths{r}];
        end
             
        
        % the robots move in 2 to 3 segments. If the robot starts with a
        % turn, that will be the first movement. Then it moves up to the 
        % second last platform of the path, then into the final position. 
        robotInput{r} = {};
        robotInput{r}{1} = 0; % first digit is turn around, only
        % necessary with hex platform
        
        if plats2turns(1) == 1  
            robotInput{r}{1} = [robotInput{r}{1}, 0, turnsLines{r}(1)];
            
            robotInput{r}{2} = 0;            
            for s = 1:length(segmentLengths{r}) - 1
                robotInput{r}{2} = [robotInput{r}{2}, ...
                    segmentLengths{r}(s), turnsLines{r}(s+1)];           
            end
            
            if segmentLengths{r}(end) > 1 
                robotInput{r}{2} = [robotInput{r}{2}, ...
                    segmentLengths{r}(end) - 1];
            end
            
            robotInput{r}{3} = [0 1];
        else
            for s = 1:length(segmentLengths{r}) - 1
                robotInput{r}{1} = [robotInput{r}{1}, ...
                    segmentLengths{r}(s), turnsLines{r}(s)];
            end
            
            if segmentLengths{r}(end) > 1 
                robotInput{r}{1} = [robotInput{r}{1}, ...
                    segmentLengths{r}(end) - 1];
            end
            
            robotInput{r}{2} = [0 1];
        end
    end
    %% determine timing of various paths
    timePerGap = 4.5;
    timePerLine = 2.5;
    
    for r = 1:2
        duration{r} = NaN(1,length(robotInput{r})-1);
        for e = 1:length(robotInput{r})-1
            turnLinesTemp = robotInput{r}{e}(3:2:end);
            turnLinesTemp(turnLinesTemp > 3) = ...
                6 - turnLinesTemp(turnLinesTemp > 3);
            
            nGapsTemp = robotInput{r}{e}(2:2:end);
            
            duration{r}(e) = (timePerGap * sum(nGapsTemp)) + ...
                (timePerLine * sum(turnLinesTemp));
        end
        duration{r}(length(robotInput{r})) = 2;
    end

    %% send ssh commands    
    nEpochs = max(cellfun(@length, duration));
    for r = 1:2
        if length(robotInput{r}) < nEpochs
            robotInput{r} = [{[]}, robotInput{r}];
            duration{r} = [0, duration{r}];
        end
    end
    
    for e = 1:nEpochs
        tic
        for r = 1:2
            if isempty(robotInput{r}{e})
                continue
            end
            
            ipAddr = robot(nonStatRobots(r)).ip;
            
            robotCom = './straight_reportIR ';
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
        while true
            elapsedTime = toc; 
            if elapsedTime > duraTemp
                break
            end
        end
    end
    %% get result
    % initially, this will be manually determined by the experimenter
    
    % figure
    newStatPlat = input(['what platform did the animal choose (1 = ' ...
        num2str(nextPlats(1)) ', 2 = ' num2str(nextPlats(2)) ')?']);
    newStatPlat = nextPlats(newStatPlat);
    
    close all
    %% update robot positions
    for r = 1:2
        robot(nonStatRobots(r)).pos = robotsFinal(r).pos;
        robot(nonStatRobots(r)).dir = robotsFinal(r).dir;
    end
    %% identify new moving robots
    if robotsFinal(1).pos == newStatPlat
        statRobot = nonStatRobots(1);
    else
        statRobot = nonStatRobots(2);
    end
    nonStatRobots = find(~ismember(1:3, statRobot));
    
    %% check if animal is at the goal
    if newStatPlat == goalPlatform
        atGoal = true;
    end
end
%% animal at goal, so drive other robots away



%%
% robotCh{1}  =  sshfrommatlabclose(robotCh{1});


























