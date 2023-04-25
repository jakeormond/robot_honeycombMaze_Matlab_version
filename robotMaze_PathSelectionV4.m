% robotMaze_PathSelection
% requires "SSH From Matlab" from Matlab file exchange; run commmand
% "javaaddpath('ganymed-ssh2-build250.jar')" from SSH From Matlab folder
%% connect to robots
% javaaddpath('ganymed-ssh2-build250.jar')
% robotCh{1}  =  sshfrommatlab('root', '192.168.0.104','');
% robotCh{2}  =  sshfrommatlab('root', '192.168.0.103','');
% robotCh{3}  =  sshfrommatlab('root', '192.168.0.106','');

%% set initial robot positions and directions
% 6 possible directions = N, NE, SE, S, SW, SE
direction.N = 0; direction.NE = 60; direction.SE = 120; directionS = 180;
direction.SW = 240; direction.NW = 300;

% robot initial positions and orientations
robot(1).pos = 68;
robot(1).dir = 300;
robot(1).ip = [192, 168 ,0, 104];

robot(2).pos = 40;
robot(2).dir = 180;
robot(2).ip = [192, 168 ,0, 103];

robot(3).pos = 58;
robot(3).dir = 180;
robot(3).ip = [192, 168 ,0, 106];

% robots moved into positions 58 and 68 (76 was the stationary platform).
% Now 58 and 76 are the moving platforms, but they're stuck! Platforms need
% to move straight in, not at an angle!

statRobot = 1; % stationary robot, other robots are to move to adjacent positions.
nonStatRobots = find(~ismember(1:3, statRobot));

goalPlatform = 100;
atGoal = false;

%% make the platform map
nRows = 18; nCols = 9;
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
while ~atGoal
    %% platform positions around stationary robot
    [rings, vertices] = getRings(robot(statRobot).pos, platformMap);
    
    %% get initial positions
    [initialPos, movFirst] = getInitPos(robot(nonStatRobots), robot(statRobot), platformMap);
    
    %% pick new platform positions
    nextPlats = getNextPlats(robot(statRobot).pos, ...
        [robot(~ismember(1:3,statRobot)).pos], platformMap, excludedPlats);
    nextPlats = [76, 86];
    nextPlats = sort(nextPlats);
    %% construct paths from initial positions to final positions
    % robots move around the rings. If they are in the stationary platform's
    % outer ring, they move along the ring until they are in position to 
    % reach the target, unless they first reach the node that is 1 before an outer ring
    % vertex; from that position, they move to the middle ring and continue
    % searching for the target.
    
    for p = 1:2
        % get inner ring of next platform, since this tells us how far to
        % travel around the middle ring
        [targetRings, targetVertices] = ...
            getRings(nextPlats(p), platformMap);
        % then find where outer ring and target ring intersect
        outerTargets = intersect(targetRings.inner, rings.middle);
        % pick the outerTarget that is lies on 1 of the 3 axes through the
        % current stationary platform. Use getDirection; if the platform
        % doesn't lie on one of the axes, get Direction will produce a NaN.
        oT = 1;
        while oT <= length(outerTargets)
            direction = getDirection(outerTargets(oT), ...
                robot(statRobot).pos, platformMap);
            if isnan(direction)
                outerTargets(oT) = [];
            else
                oT = oT + 1;
            end
        end
        
        
        for r = 1:2
            % determine if platform is in middle or outer ring.
            middleRingFlag = false;
            if ismember(initialPos{r}(end), rings.middle) % in middle ring
                middleRingFlag = true;
            end
            
            % middle ring is the simple case, just follow the ring around
            if middleRingFlag
                for d = 1:2 % direction
                    % is in middle ring, so just travel along middle ring
                    pathTemp = [rings.middle; rings.middle];
                    if d == 2
                        pathTemp = flipud(pathTemp);
                    end
                    pathStart = find(pathTemp == initialPos{r}(end),1);
                    pathTemp = pathTemp(pathStart:end);
                    [~,pathEnd] = intersect(pathTemp, outerTargets);
                    
                    if length(initialPos{r}) > 1
                        pathTemp = [initialPos{r}(1:end-1); ...
                            pathTemp(1:min(pathEnd)); nextPlats(p)];
                    else
                        pathTemp = [pathTemp(1:min(pathEnd)); nextPlats(p)];
                    end
                    
                    pathFromInit{r, p, d} = pathTemp;
                end
                
                % more complicated if in the outer ring
                % NEED TO CHECK: 1. Is the robot on a middle vertex of the target,
                % if so it can go straight there. 2. At last position before outer
                % ring vertices, robot can jump to middle ring.
                
            else % platform is in outer ring
                for d = 1:2
                    if ismember(initialPos{r}(end), targetVertices.middle) && ...
                            ~checkAdjacent(initialPos{1}(end), initialPos{2}(end), ...
                            platformMap)
                        
                        % can go directly there, just need to get the next
                        % position
                        ringsTemp = getRings(initialPos{r}(end), platformMap);
                        nextPos = intersect(ringsTemp.inner, targetRings.inner);
                        pathFromInit{r, p, d} = [initialPos{r}; ...
                            nextPos; nextPlats(p)];
                        
                    else
                        % go around outer ring, but move to middle ring at
                        % last node before outer ring vertex. While in outer
                        % node, check that you haven't hit the target's middle
                        % ring vertex
                        counter = 1;
                        pathTemp = {};
                        pathTemp{1} = initialPos{r};
                        ring = 'outer';
                        
                        while true
                            counter = counter + 1;
                            pathTemp{counter} = [rings.(ring); rings.(ring)];
                            
                            if d == 2
                                pathTemp{counter} = flipud(pathTemp{counter});
                            end
                            
                            pathStart = find(pathTemp{counter} == ...
                                pathTemp{counter-1}(end),1);
                            
                            pathTemp{counter} = pathTemp{counter}(pathStart+1:end);
                            
                            if strcmp(ring, 'outer')
                                % find first vertex
                                [~, vertInd] = intersect(pathTemp{counter}, vertices.outer);
                                vertInd = sort(vertInd);
                                % can only skip the vertex if it was not part of the
                                % initial position.
                                
                                if ~ismember(pathTemp{counter}(vertInd(1)), initialPos{r})
                                    % remove vertex AND jump to middle ring. The first
                                    % middle ring platform will be the only middle
                                    % ring vertex adjacent to the outer vertex
                                    [vertRings, ~] = getRings(pathTemp{counter}(vertInd(1)), platformMap);
                                    firstMiddle = intersect(vertRings.inner, vertices.middle);
                                    
                                    pathTemp{counter} = [pathTemp{counter}(1:vertInd(1)-1); ...
                                        firstMiddle];
                                    ring = 'middle';
                                    % add the first position from the middle ring
                                    
                                else
                                    pathTemp{counter} = pathTemp{counter}(1:vertInd(1));
                                end
                                
                                % determine if outer ring path intesects any target
                                % middle vertices
                                [~, targetVertInd] = intersect(pathTemp{counter}, targetVertices.middle);

                                if ~isempty(targetVertInd)
                                    targetVertInd = targetVertInd(1);
                                    
                                    pathTemp{counter} = pathTemp{counter}(1:targetVertInd);
                                    
                                    % now construct path to target
                                    straightPath = getStraigthPath(pathTemp{counter}(end), ...
                                        nextPlats(p), platformMap);
                                    
                                    pathFromInit{r, p, d} = ...
                                        [vertcat(pathTemp{:}); straightPath(2:end)];
                                    break
                                end
                            end
                            
                            [~,pathEnd] = intersect(pathTemp{counter}, outerTargets);
                            pathEnd = sort(pathEnd);
                            if ~isempty(pathEnd)
                                pathTemp{counter} = [pathTemp{counter}(1:pathEnd); ...
                                    nextPlats(p)];
                                pathFromInit{r, p, d} = vertcat(pathTemp{:});
                                break
                            end
                        end
                    end
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
    
    compMatrix = false(2,2,2);
    staggerMatrix = zeros(2,2,2);
    pathsTemp = cell(1,2);
    for p1 = 1:2 % robot1's platform, which determines robot2's platform
        p2 = find(~ismember([1 2], p1));
        
        for d1 = 1:2 % robot1's direction
            pathsTemp{1} = pathFromInit{1, p1, d1};
            startInd1 = find(pathsTemp{1} == initialPos{1}(end));
            pathsTemp{1} = pathsTemp{1}(startInd1:end);
            
            for d2 = 1:2 % robot2's direction                
                pathsTemp{2} = pathFromInit{2, p2, d2};
                startInd2 = find(pathsTemp{2} == initialPos{2}(end));
                pathsTemp{2} = pathsTemp{2}(startInd2:end);
                
                [compFlag, stagger] = checkPathCompatibility(pathsTemp, ...
                    movFirst, platformMap);
                
                compMatrix(p1, d1, d2) = compFlag;
                staggerMatrix(p1, d1, d2) = stagger;                
            end
        end
    end
    %% calculate final paths using compatibility matrix and path lengths
    maxLengths = NaN(2,2,2);
    for p1 = 1:2 % robot1's platform, which determines robot2's platform
        p2 = find(~ismember([1 2], p1));
        
        for d1 = 1:2 % robot1's direction
            pathsTemp{1} = pathFromInit{1, p1, d1};
            startInd1 = find(pathsTemp{1} == initialPos{1}(end));
            pathsTemp{1} = pathsTemp{1}(startInd1:end);
            
            for d2 = 1:2 % robot2's direction                
                pathsTemp{2} = pathFromInit{2, p2, d2};
                startInd2 = find(pathsTemp{2} == initialPos{2}(end));
                pathsTemp{2} = pathsTemp{2}(startInd2:end);
                
                if ~compMatrix(p1, d1, d2)                    
                    continue
                
                else
                    maxLengths(p1, d1, d2) = max(length(pathsTemp{1}), ...
                        length(pathsTemp{2}));                    
                end              
            end
        end
    end
    % maxLengths = maxLengths + staggerMatrix;
    [~, minInd] = min(maxLengths(:));
    [p1, d1, d2] = ind2sub(size(maxLengths), minInd);
    pathsFinal{1} = pathFromInit{1,p1,d1};
    p2 = find(~ismember([1 2], p1));
    pathsFinal{2} = pathFromInit{2,p2,d2};
    
    stagger = staggerMatrix(p1, d1, d2);
    %%
    % calculate distances for 2 possible mappings of initial positions to final
    % positions
    % first if r1 goes to p1 and r2 goes to p2
%     distance1_1(1) = length(pathFromInit{1,1,1}) ;
%     distance1_1(2) = length(pathFromInit{1,1,2});
%     distance2_2(1) = length(pathFromInit{2,2,1});
%     distance2_2(2) = length(pathFromInit{2,2,2});
%        
%     option1_max = max(min(distance1_1), min(distance2_2));
%     
%     distance1_2(1) = length(pathFromInit{1,2,1});
%     distance1_2(2) = length(pathFromInit{1,2,2});
%     distance2_1(1) = length(pathFromInit{2,1,1});
%     distance2_1(2) = length(pathFromInit{2,1,2});
%     
%     option2_max = max(min(distance1_2), min(distance2_1));
% 
%     if option1_max < option2_max
%         if distance1_1(1) <= distance1_1(2)
%             pathsFinal{1} = pathFromInit{1,1,1};
%         else
%             pathsFinal{1} = pathFromInit{1,1,2};
%         end
%         
%         if distance2_2(1) <= distance2_2(2)
%             pathsFinal{2} = pathFromInit{2,2,1};
%         else
%             pathsFinal{2} = pathFromInit{2,2,2};
%         end
%     else
%         if distance1_2(1) <= distance1_2(2)
%             pathsFinal{1} = pathFromInit{1,2,1};
%         else
%             pathsFinal{1} = pathFromInit{1,2,2};
%         end
%         
%         if distance2_1(1) <= distance2_1(2)
%             pathsFinal{2} = pathFromInit{2,1,1};
%         else
%             pathsFinal{2} = pathFromInit{2,1,2};
%         end
%         
%     end
    %% add start platform to path if not already there
    for r = 1:2
        if robot(nonStatRobots(r)).pos ~= pathsFinal{r}(1)
            pathsFinal{r} = [robot(nonStatRobots(r)).pos; pathsFinal{r}];
        end
    end
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
    %% check if robots need to be staggered
%     stagger = NaN;
%     if movFirst == 1 || movFirst == 2 % check if they are staggered
%         movSecond = ~ismember([1,2], movFirst);
%         
%         for p = 1:length(pathsFinal{movFirst})-1
%             pathFirst = pathsFinal{movFirst}(p+1:end);
%             pathSecond = pathsFinal{movSecond};
%             if length(pathSecond) > length(pathFirst)
%                 pathSecond = pathSecond(1:length(pathFirst));
%             end
%             
%             minLength = min(length(pathFirst), length(pathSecond));
%             
%             adjFlag = false(minLength, 1);
%             for p2 = 1:minLength
%                 if checkAdjacent(pathFirst(p2), pathSecond(p2), ...
%                         platformMap) || pathFirst(p2) == pathSecond(p2)
%                     
%                     adjFlag(p2) = true;
%                 end
%             end
%             
%             if isempty(find(adjFlag, 1))
%                 stagger = p;
%                 break
%             end
%         end
%         
%         if isnan(stagger)
%             error
%         end
%     end
    
    %% construct robot inputs path
    % this consists of the sequence of linear motions (defined by number of gaps to
    % cross) interleaved with turns (defined by number of lines to cross)
    % if a robot is to move second, it still needs to move half a
    % position away as soon as the first robot has moved some distance.
    movSecond = find(~ismember([1,2], movFirst));
    turnAround = zeros(1,2);
    for r = 1:2
        currDir = robot(nonStatRobots(r)).dir;
        directions = NaN(length(pathsFinal{r})-1, 1);
        for p = 1:length(pathsFinal{r})-1
            % get direction to next platform
            currPlat = pathsFinal{r}(p);
            nextPlat = pathsFinal{r}(p+1);
            directions(p) = getDirection(currPlat, nextPlat, platformMap);
        end
        robotsFinal(r).pos = pathsFinal{r}(end);
        robotsFinal(r).dir = directions(p);
        
        if directions(1) ~= currDir
            turnAround(r) = 1;
        end
        
        % find turns
        directionDiff = diff(directions);
        plats2turns = find(directionDiff ~= 0);
        nSegments = length(plats2turns) + 1;
        segmentLengths{r} = [plats2turns(1); diff(plats2turns); ...
            (length(pathsFinal{r})-1) - plats2turns(end)];
        turns = directionDiff(directionDiff~=0);
        turns(turns < 0) = turns(turns < 0) + 360;
        turnsLines{r} = turns/60;
        
        if r == movSecond % there will be 3 segments: 1) the first half-move,
            % 2) the remainder up until 1 platform before target, 3) final move
            % to target
            robotInput{r} = cell(1,3);
            robotInput{r}{1} = [turnAround(r), 97]; % will have to rewrite
            % robot program to interpret 97 as move to the gap without crossing
                      
            robotInput{r}{2} = NaN(1, nSegments*2);
            robotInput{r}{2}(1) = 0;
            robotInput{r}{2}(2) = 98; % 98 will be to complete 97 across the gap.
            % Also, for a robot that moves second, first move will always only
            % be length == 1
            for s = 1:nSegments-1
                robotInput{r}{2}(s*2 + 1) = turnsLines{r}(s);
                
                if s ~= nSegments-1
                    robotInput{r}{2}(s*2 + 2) = segmentLengths{r}(s+1);
                    
                elseif segmentLengths{r}(end) > 1
                    robotInput{r}{2}(s*2 + 2) = segmentLengths{r}(end) - 1;
                end
            end
            if isnan(robotInput{r}{2}(end))
                robotInput{r}{2}(end) = [];
            end
            
            robotInput{r}{3} = [0 1];
            
        else % there will be 2 segments: 1) everything but final move to
            % target, 2) final move to target
            if segmentLengths{r}(end) == 1
                robotInput{r}{1} = NaN(1, nSegments*2 - 1);
            else
                robotInput{r}{1} = NaN(1, nSegments*2);
            end
            
            robotInput{r}{1}(1) = turnAround(r);
            
            for s = 1:nSegments-1
                robotInput{r}{1}(s*2) = segmentLengths{r}(s);
                robotInput{r}{1}(s*2 + 1) = turnsLines{r}(s);
            end
            if segmentLengths{r}(end) ~= 1
                robotInput{r}{1}(nSegments*2) = segmentLengths{r}(end)-1;
            end
            
            robotInput{r}{2} = [0 1];
        end
    end
    %% determine timing of various paths
    timePerGap = 5;
    timePerLine = 2;
    timing = cell(1,2);
    
    if movFirst > 0
        timing{movFirst}(1) = 0;
        timing{movSecond}(1) = 6; % in seconds
        
        if stagger > 1
            nLinesPerStagger = 0;
            for s = 1:stagger
                if s < stagger
                    nLinesTemp = robotInput{movFirst}{1}(s*2 + 1);
                    if nLinesTemp > 3
                        nLinesTemp = 6-nLinesTemp;
                    end
                    nLinesPerStagger = nLinesPerStagger + nLinesTemp;
                end
            end
            
            % some formula to convert gaps and lines to time in seconds
            timing{movSecond}(2) = stagger*timePerGap + nLinesPerStagger*timePerLine;
            
            turnsFirst = turnsLines{movFirst};
            turnsFirst(turnsFirst > 3) = 6-turnsFirst(turnsFirst > 3);
            totalTimeFirst = sum(segmentLengths{movFirst})*timePerGap + sum(turnsFirst)*timePerLine;
            
            turnsSecond = robotInput{movSecond}{2}(3:2:end);
            turnsSecond(turnsSecond > 3) = 6-turnsSecond(turnsSecond > 3);
            nSegsTemp = robotInput{movSecond}{2}(2:2:end);
            nSegsTemp(nSegsTemp == 98) = 1;
            totalTimeSecond = sum(nSegsTemp)*timePerGap + sum(turnsSecond)*timePerLine;
            totalTimeSecond  = totalTimeSecond + timing{movSecond}(2);
            
            delay2end = max(totalTimeFirst, totalTimeSecond);
            
            timing{movSecond}(3) = delay2end;
            timing{movFirst}(2) = delay2end;
            
        else
            turnsFirst = turnsLines{1};
            turnsFirst(turnsFirst > 3) = 6-turnsFirst(turnsFirst > 3);
            totalTimeFirst = sum(segmentLengths{1})*timePerGap + sum(turnsFirst)*timePerLine;
            
            turnsSecond = turnsLines{2};
            turnsSecond(turnsSecond > 3) = 6-turnsSecond(turnsSecond > 3);
            totalTimeSecond = sum(segmentLengths{2})*timePerGap + sum(turnsSecond)*timePerLine + ...
                timing{movSecond}(1);
            
            delay2end = max(totalTimeFirst, totalTimeSecond);
            
            timing{1}(2) = delay2end;
            timing{2}(2) = delay2end;
            
            robotInput{movSecond}{1} = [robotInput{movSecond}{1}(1), ...
                1, robotInput{movSecond}{2}(3:end)];
            robotInput{movSecond}(2) = [];
        end
        
    else
        timing{1}(1) = 0;
        timing{2}(1) = 0;
        
        turnsFirst = turnsLines{1};
        turnsFirst(turnsFirst > 3) = 6-turnsFirst(turnsFirst > 3);
        totalTimeFirst = sum(segmentLengths{1})*timePerGap + sum(turnsFirst)*timePerLine;
        
        turnsSecond = turnsLines{2};
        turnsSecond(turnsSecond > 3) = 6-turnsSecond(turnsSecond > 3);
        totalTimeSecond = sum(segmentLengths{2})*timePerGap + sum(turnsSecond)*timePerLine;
        
        delay2end = max(totalTimeFirst, totalTimeSecond);
        
        timing{1}(2) = delay2end;
        timing{2}(2) = delay2end;
    end
    %% send ssh commands
    
    timingTemp = timing;
    robotInputTemp = robotInput;
    
    tic
    finished = false(1,2);
    while true
        for r = 1:2
            if finished(r)
                continue
            end
            
            elapsedTime = toc; % in seconds
            if timingTemp{r}(1) > elapsedTime
                continue
            end
            
            timingTemp{r}(1) = [];
            if isempty(timingTemp{r})
                finished(r) = true;
            end
            
            ipAddr = robot(nonStatRobots(r)).ip;
            
            robotCom = './lineFollowJunction9 ';
            robotComVar = robotInputTemp{r}{1};
            
            for v = 1:length(robotComVar)
                if v < length(robotComVar)
                    robotCom = [robotCom num2str(robotComVar(v)) ' '];
                else
                    robotCom = [robotCom num2str(robotComVar(v))];
                end
            end
            robotInputTemp{r}(1) = [];
            
            robotID = nonStatRobots(r);
            robotCh{robotID} =  ...
                sshfrommatlabissue_dontwait(robotCh{robotID}, robotCom);
            
            elapsedTime = round(elapsedTime, 1);
            fprintf('elapsed time is %3.1f seconds\n', elapsedTime)
            ipStr = num2str(ipAddr(1));
            for i = 2:length(ipAddr)
                ipStr = [ipStr '.' num2str(ipAddr(i))];
            end
            
            fprintf(['robot ' ipStr ' command: ' robotCom '\n'], nonStatRobots(r))
            
            pause(0.3)
        end
        
        if length(find(finished)) > 1
            break
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


























