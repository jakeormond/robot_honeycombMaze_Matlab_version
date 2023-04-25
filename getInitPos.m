function [initPos, movFirst] = getInitPos(movRobot, statRobot, platformMap)
% get the next 2 possible positions for a robot given it's current position
% and orientation. It assumes the robot can't turn due to being adjacent to
% another robot.
% possPos has 2 values: first, the position in front, second, the position
% behind
%% first, determine if either robot is outside the statRobot's inner ring
% (i.e. in its middle ring), since this means it may not have to move. 
movFirst = 0;
[rings, ~] = getRings(statRobot.pos, platformMap);

innerRingFlag = false(1,2);
for r = 1:length(movRobot)
    if ismember(movRobot(r).pos, rings.inner)
        innerRingFlag(r) = true;
    end
end

initPos = cell(1,2);
for r = 1:length(movRobot)
    if ~innerRingFlag(r) 
        initPos{r} = movRobot(r).pos;
        continue % we deal with the outer ring platform below
    end    
    
    possPosTemp = get2PossPos(movRobot(r).pos, ...
        movRobot(r).dir, platformMap);
      
    % one of these 2 possible positions may be located in the inner ring of
    % the stationary robot or one top of it, so must be excluded. 
    [~, intInd] = intersect(possPosTemp, [rings.inner; statRobot.pos]);
    possPosTemp(intInd) = [];
    initPos{r} = possPosTemp;
end
%% check if initial positions are compatible. They cannot be members of 
% each other's inner ring (i.e. adjacent)
if (initPos{1} == initPos{2} || checkAdjacent(initPos{1}, ...
        initPos{2}, platformMap)) && ~isempty(find(~innerRingFlag,1))
    movFirst = find(~innerRingFlag); % if 1 of the robots starts in the middle
    movSecond = find(~ismember([1 2], movFirst));
    
    possPosTemp = get2PossPos(movRobot(movFirst).pos, ...
        movRobot(movFirst).dir, platformMap);
    
    if possPosTemp(1) == movRobot(movSecond).pos || ...
            checkAdjacent(possPosTemp(1), movRobot(movSecond).pos, platformMap)
        possPosTemp(1) = [];
    else
        possPosTemp(2) = [];
    end
    initPos{movFirst} = [initPos{movFirst}; possPosTemp];
    
    % check again whether these initial positions are compatible. If
    % they are not, the outer robot needs to move 1 position around
    % the outer ring; it can only do so in 1 direction.
    if checkAdjacent(initPos{movFirst}(end), initPos{movSecond}, platformMap)
        outerRingInd = find(rings.outer == initPos{movFirst}(end));
        
        if outerRingInd == 1
            possPosTemp = [rings.outer(end), ...
                rings.outer(outerRingInd+1)];
            
        elseif outerRingInd == length(rings.outer)
            possPosTemp = [rings.outer(outerRingInd-1), ...
                rings.outer(1)];
        else
            possPosTemp = [rings.outer(outerRingInd-1), ...
                rings.outer(outerRingInd+1)];
        end
        
        adjacent1 = checkAdjacent(possPosTemp(1), initPos{innerRingFlag}, ...
            platformMap);
        
        adjacent2 = checkAdjacent(possPosTemp(2), initPos{innerRingFlag}, ...
            platformMap);
            
        if adjacent1 && ~adjacent2
            possPosTemp(1) = [];
        elseif adjacent2 && ~adjacent1
            possPosTemp(2) = [];
        else
            possPosTemp = [];
        end
        initPos{movFirst} = [initPos{movFirst}; possPosTemp];
    end
    
else
    % check if robot 2's new position is in robot 1's pre-move inner
    % ring. If it is, move robot 1 first
    [ringsMov1, ~] = getRings(movRobot(1).pos, platformMap);
    [ringsMov2, ~] = getRings(movRobot(2).pos, platformMap);
    if ismember(initPos{2}(end), ringsMov1.inner)
        movFirst = 1;
    elseif ismember(initPos{1}(end), ringsMov2.inner)
        movFirst = 2;
    end
    movSecond = find(~ismember([1 2], movFirst));
    % check if initial positions are compatible
    if movFirst > 0
        if checkAdjacent(initPos{movFirst}(end), initPos{movSecond}(end), ...
                platformMap)
            % moveFirst robot needs to move 1 more node
            ringInd = find(rings.middle == initPos{movFirst}(end));
            if ringInd == 1
                nextPlats = [rings.middle(end), rings.middle(2)];
                
            elseif ringInd == length(rings.middle)
                nextPlats = [rings.middle(1), rings.middle(end-1)];

            else
                nextPlats = [rings.middle(ringInd-1), rings.middle(ringInd+1)];
            end
            
            if nextPlats(1) == initPos{movSecond}(end)
                nextPlats(1) = [];
            else
                nextPlats(2) = [];
            end
            initPos{movFirst} = [initPos{movFirst}; nextPlats];
        end
    end
end
end








