% robotMaze_PathSelection_CircularPlatforms
% requires "SSH From Matlab" from Matlab file exchange; run commmand
% "javaaddpath('ganymed-ssh2-build250.jar')" from SSH From Matlab folder
%% connect to robots
% rootdir = cd;
% cd('C:\Users\LabUser\odrive\Amazon Cloud Drive\Documents\matlab_code\file_exchange_code\sshfrommatlab_13b') 
% javaaddpath('ganymed-ssh2-build250.jar')
% robotCh{1}  =  sshfrommatlab('root', '192.168.0.101','');
% robotCh{2}  =  sshfrommatlab('root', '192.168.0.103','');
% robotCh{3}  =  sshfrommatlab('root', '192.168.0.106','');
% cd(rootdir)
%% set data storage
% rat = 'Rat11';
% currDate = datestr(date, 'dd-mm-yyyy');
% if ~exist(currDate, 'dir')
%     mkdir(currDate)    
% end
% cd(currDate)

currTime = clock; currTime = currTime(4:6);
currTimeStr = [num2str(currTime(1)), num2str(currTime(2))];
fileName = ['trial_' currTimeStr '.mat'];

if exist('choices.mat', 'file')
    load choices.mat
else
    choiceCounterAll = 0;
    choicesAll = [];    
end

choiceCounter = 0;
choices = [];
%% make the platform map
nRows = 26; nCols = [9, 10];
platformMap = makePlatMap(nRows, nCols);
restrictedMap = platformMap;
%% set initial robot positions and directions
% 6 possible directions = N, NE, SE, S, SW, SE
direction.N = 0; direction.NE = 60; direction.SE = 120; directionS = 180;
direction.SW = 240; direction.NW = 300;

% set starting robot, default will be robot 1
statRobot = 1; % stationary robot, other robots are to move to adjacent positions.
nonStatRobots = find(~ismember(1:3, statRobot));

% robot initial positions and orientations
robot(1).pos = 103;
robot(1).dir = 180;
robot(1).ip = [192, 168 ,0, 101];

robot(2).pos = 65; % robots 2 and 3 should be placed 1 spot away from this position
robot(2).dir = 180; % so that they can be moved into position once program is started.
robot(2).ip = [192, 168 ,0, 103];

robot(3).pos = 141;
robot(3).dir = 0;
robot(3).ip = [192, 168 ,0, 106];

allPlatPos(1).robot1_pos = robot(1).pos;
allPlatPos(1).robot1_dir = robot(1).dir;
allPlatPos(1).robot2_pos = robot(2).pos;
allPlatPos(1).robot2_dir = robot(2).dir;
allPlatPos(1).robot3_pos = robot(3).pos;
allPlatPos(1).robot3_dir = robot(3).dir;


% goal platform
goalPlatform = 187;

% spin flag, used to get animal moving
spinFlag = false;

%% make list of platform positions that can't be used for final positions.
% These positions are excluded because it's impossible to navigate robots
% close to walls
excludedPlatsCell{1} = platformMap(1:4, :); restrictedMap(1:4, :) = [];
excludedPlatsCell{2} = platformMap(:, 1:5); restrictedMap(:, 1:5) = [];
excludedPlatsCell{3} = platformMap(end-3:end, :); restrictedMap(end-3:end, :) = [];
excludedPlatsCell{4} = platformMap(:, end-2:end); restrictedMap(:, end-2:end) = [];

for i = 1:4
    excludedPlatsCell{i} = excludedPlatsCell{i}(:);
end
excludedPlats = unique(vertcat(excludedPlatsCell{:}));
excludedPlats(isnan(excludedPlats)) = [];

%% permanent variables

timePerGap = 3.5;
timePerLine = 1.5;

duraSpin = 5;

% difficulty = 'easy';
difficulty = 'hard';

atGoal = false;
%% trial runs within a loop
% choices(1).currentPlatform = robot(statRobot).pos;
% choices(1).choicePlatforms = [0, 0];

% choiceCounter = 0;

loopCounter = 0;

while ~atGoal
    loopCounter = loopCounter+1;
    %% pick new platform positions
    % find previous combinations of platforms, if any, previously chosen
    % from that location.
    % NOTE: SHOULD IDENTIFY HERE IF ALL POSSIBLE CHOICES HAVE BEEN GIVEN,
    % IN WHICH CASE, CHOICES WILL HAVE TO BE REPEATED.....NOT CODED YET!
    
    %     prevVisits = find([choices(:).currentPlatform] == robot(statRobot).pos);
    %     if ~isempty(prevVisits)
    %         prevChoices = vertcat(choices(prevVisits).choicePlatforms);
    %     else
    %         prevChoices = [];
    %     end
    
    % get distance to goal of current platform
    currDistance = cartesianDistance(robot(statRobot).pos, ...
        goalPlatform, platformMap);
    
    if currDistance == 0 % animal has reached the goal
        atGoal = true;
        % need to pick next platforms in a different way, since we want
        % them away from the stationary platform, not adjacent to it.
        % 3 possibilities: 1) both moving platforms are adjacent to the
        % goal, therefore both need to move to the middle ring; 2) only one
        % moving platform is adjacent to the goal, and it can move freely
        % to the middle ring; 3) only one moving platform is adjacent to
        % the goal, but it can't move freely to the middle ring, so the
        % second platform also needs to move;
        [initialPos, ~] = ...
            getInitPos_CircularPlatforms(robot(nonStatRobots), ...
            robot(statRobot), platformMap);
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
    else
        while true
            nextPlats = getNextPlats(robot(statRobot).pos, ...
                [robot(~ismember(1:3,statRobot)).pos], platformMap, excludedPlats);
            % nextPlats = [90, 100];
            nextPlats = sort(nextPlats);
            
            %         if ~isempty(prevChoices)
            %             if ~isempty(find(prevChoices(:,1) == nextPlats(1) && ...
            %                     prevChoices(:,2) == nextPlats(2), 1))
            %                 continue
            %             end
            %         end
            
            % calculate distance of all platforms to the goal. At least one of
            % the offered platforms need to be closer to the goal than the
            % animal's current location if difficulty set to 'hard'; both must
            % be closer if set to 'easy', unless at the goal.
            dist1 = cartesianDistance(nextPlats(1), ...
                goalPlatform, platformMap);
            dist2 = cartesianDistance(nextPlats(2), ...
                goalPlatform, platformMap);
            
            if (strcmp(difficulty, 'easy') && dist1 < currDistance ...
                    && dist2 < currDistance) || (strcmp(difficulty, 'hard') ...
                    && (dist1 < currDistance || dist2 < currDistance)) || ...
                    (strcmp(difficulty, 'easy') && (dist1 == 0 || dist2 == 0))
                
                break
            end
        end
    end
    %     choiceCounter = choiceCounter+1;
    %     choices(choiceCounter).choicePlatforms = nextPlats;
    
    %%
    if ~atGoal
        
        
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
                    pathsTemp{1} = [robot(nonStatRobots(1)).pos; ...
                        paths{1, p1, i1, d1}];
                    
                    for i2 = 1:length(initialPos{2})
                        for d2 = 1:2 % robot2's direction
                            
                            if movFirst > 0
                                if d2 == d1
                                    continue % robots must move in opposite directions in this specific condition
                                end
                            end
                            
                            pathsTemp{2} = [robot(nonStatRobots(2)).pos; ...
                                paths{2, p2, i2, d2}];
                            
                            compFlag = checkPathCompatibilityV2(pathsTemp, ...
                                platformMap, d1, d2);
                            
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
            p1 = p1(minInd);
            i1 = i1(minInd);
            d1 = d1(minInd);
            i2 = i2(minInd);
            d2 = d2(minInd);
        end
        
        p2 = find(~ismember([1 2], p1));
        
        pathsFinal{1} = [robot(nonStatRobots(1)).pos; ...
            paths{1, p1, i1, d1}; nextPlats(p1)];
        pathsFinal{2} = [robot(nonStatRobots(2)).pos; ...
            paths{2, p2, i2, d2}; nextPlats(p2)];
        
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
            nSegments = length(plats2turns); % nSegments may be unused
            
            segmentLengths = diff([plats2turns; length(pathsFinal{r})]);
            
            turns = directionDiff(directionDiff~=0);
            turns(turns < 0) = turns(turns < 0) + 360;
            turnsLines = turns/60;
            
            if isempty(plats2turns) || plats2turns(1) ~= 1
                nSegments = nSegments + 1; % nSegments may be unused
            end
            
            if isempty(plats2turns)
                segmentLengths = length(pathsFinal{r}) - 1;
            elseif plats2turns(1) ~= 1
                segmentLengths = [plats2turns(1)-1; segmentLengths];
            end
            
            
            % the robots move in 2 to 3 segments. If the robot starts with a
            % turn, that will be the first movement. Then it moves up to the
            % second last platform of the path, then into the final position.
            robotInput{r} = {};
            robotInput{r}{1} = 0; % first digit is turn around, only
            % necessary with hex platform
            
            if isempty(plats2turns)
                if segmentLengths == 1
                    robotInput{r}{1} = [robotInput{r}{1}, 1];
                else
                    robotInput{r}{1} = [robotInput{r}{1}, segmentLengths -1];
                    robotInput{r}{2} = [0 1];
                end
                
            elseif plats2turns(1) == 1
                robotInput{r}{1} = [robotInput{r}{1}, 0, turnsLines(1)];
                
                robotInput{r}{2} = 0;
                for s = 1:length(segmentLengths) - 1
                    robotInput{r}{2} = [robotInput{r}{2}, ...
                        segmentLengths(s), turnsLines(s+1)];
                end
                
                if segmentLengths(end) > 1
                    robotInput{r}{2} = [robotInput{r}{2}, ...
                        segmentLengths(end) - 1];
                end
                
                robotInput{r}{3} = [0 1];
            else
                for s = 1:length(segmentLengths) - 1
                    robotInput{r}{1} = [robotInput{r}{1}, ...
                        segmentLengths(s), turnsLines(s)];
                end
                
                if segmentLengths(end) > 1
                    robotInput{r}{1} = [robotInput{r}{1}, ...
                        segmentLengths(end) - 1];
                end
                
                robotInput{r}{2} = [0 1];
            end
        end
        
    else % construct simple path for robots moving away from goal at trial end
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
    end
    %% determine timing of various paths
    duration = cell(1,2);
    for r = 1:2
        if isempty(robotInput{r})
            continue
        end
        
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
    
    nEpochs = max(cellfun(@length, duration));
    for r = 1:2
        while length(robotInput{r}) < nEpochs
            robotInput{r} = [{[]}, robotInput{r}];
            duration{r} = [0, duration{r}];
        end
    end
    %% send ssh commands
    decChange = [];
    for e = 1:nEpochs
        tic
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
        while true
            elapsedTime = toc;
            if elapsedTime > duraTemp
                break
            end
        end
        
        if (e == 1 && nEpochs == 3) || (e == 1 && atGoal) % if we are on a turn-only step
            
            decChange = input('did the animal change it''s decision? (''y'', hit enter for no])\n', 's');
            
            if decChange == 'y' % need to update robot directions and exit for loop
                
                if atGoal
                    atGoal = false;
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
                    newDirection = robot(nonStatRobots(r)).dir + degreesTurned;
                    if newDirection < 0
                        newDirection = newDirection + 360;
                    elseif newDirection >= 360
                        newDirection = newDirection - 360;
                    end
                    
                    robot(nonStatRobots(r)).dir = newDirection;
                    
                    allPlatPos(loopCounter+1).(['robot' num2str(nonStatRobots(r)) '_dir']) ...
                        = newDirection;

                    
                end
                choiceCounter = choiceCounter - 1;
                statRobot = statRobotOld;
                nextPlats = nextPlatsOld;                
                break
            end
        end
    end
    
    % update robot
    if ~strcmp(decChange, 'y')
        for r = 1:2
            robot(nonStatRobots(r)).pos = robotsFinal(r).pos;
            robot(nonStatRobots(r)).dir = robotsFinal(r).dir;
            
            allPlatPos(loopCounter+1).(['robot' num2str(nonStatRobots(r)) '_pos']) ...
                = robotsFinal(r).pos;
            allPlatPos(loopCounter+1).(['robot' num2str(nonStatRobots(r)) '_dir']) ...
                = robotsFinal(r).dir;
        end
    end
    
    %% get result
    if ~atGoal
        
        % figure
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
        
        choiceCounter = choiceCounter + 1;
        currTime = clock; currTime = currTime(4:6);
        choices(choiceCounter, 1) = currTime(1);
        choices(choiceCounter, 2) = currTime(2);
        choices(choiceCounter, 3) = currTime(3);
        choices(choiceCounter, 4) = robot(statRobot).pos;
        choices(choiceCounter, 5) = newStatPlat;
        choices(choiceCounter, 6) = nextPlats(nextPlats ~= newStatPlat);
        
        statRobotOld = statRobot;
        nextPlatsOld = nextPlats;
        
        close all
        %% update robot directions if animal made decision, but not if he changed his mind
%         if ~strcmp(decChange, 'y') % if the animal finalized its decision
%             for r = 1:2
%                 robot(nonStatRobots(r)).pos = robotsFinal(r).pos;
%                 robot(nonStatRobots(r)).dir = robotsFinal(r).dir;
%             end
%         end
        %% identify new moving robots
        for r = 1:3
            if robot(r).pos == newStatPlat
                statRobot = r;
                break
            end
        end
        
        nonStatRobots = find(~ismember(1:3, statRobot));
    end
end

choicesAll = [choicesAll; choices];
choiceCounterAll = choiceCounterAll + choiceCounter;
save choices.mat choicesAll choiceCounterAll
save(fileName, 'choices')
























