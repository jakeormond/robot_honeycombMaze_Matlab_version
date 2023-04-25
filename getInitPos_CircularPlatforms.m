function [initPos, movFirst] = getInitPos_CircularPlatforms(robot, platformMap)
% 
%% first, determine if either robot is outside the statRobot's inner ring
% (i.e. in its middle ring), since this means it may not have to move. 
statRobot = find(strcmp({robot(:).movState}, 'stationary'));
statRobot = robot(statRobot);

movRobot = find(strcmp({robot(:).movState}, 'moving'));
movRobot = robot(movRobot);


[rings, ~] = getRings(statRobot.pos, platformMap);
movPos = [movRobot(:).pos];

innerRingFlag = false(1,2);
initPos = cell(1,2);
for r = 1:length(movPos)
    if ismember(movPos(r), rings.inner)
        innerRingFlag(r) = true;
    end
    if ~innerRingFlag(r) % platform not in middle ring
        middleInd = find(rings.middle == movPos(r));
        if middleInd == 1
            initPos{r} = [rings.middle(end); rings.middle(1:2)];
        elseif middleInd == length(rings.middle)
            initPos{r} = [rings.middle(end-1:end); rings.middle(1)];
        else
            initPos{r} = rings.middle(middleInd-1:middleInd+1);
        end
        
    else % platform needs to move to middle ring
        % first get all adjacent platforms
        platRings = getRings(movPos(r), platformMap);
        adjacentPlats = platRings.inner;
        initPos{r} = intersect(adjacentPlats, rings.middle);
    end
    
end

nInnerRing = length(find(innerRingFlag));
% if all 3 platforms lie on a straight line, and the stationary platform is
% at one end of the line, then the middle platform cannot move until the
% other non-stationary platform moves to the outer ring. This will be the
% case if the middle platform's possible initial positions are either
% adjacent or lie overtop of the other non-stationary platform. 
movFirst = 0;
if nInnerRing == 1
    innerMovInd = find(innerRingFlag);
    outerMovInd = setdiff([1, 2], innerMovInd);
    outPlatRings = getRings(movPos(outerMovInd), platformMap);
    overlapPos = intersect(initPos{innerMovInd}, ...
        [outPlatRings.inner; movPos(outerMovInd)]); 
  
    if length(overlapPos) == length(initPos{innerMovInd}) % then outer plat needs to move to outer rin
        initPos{innerMovInd}(initPos{innerMovInd} == movPos(outerMovInd)) = [];
        initPos{outerMovInd} = [];
        
        % platform needs to move to middle ring
        initPos{outerMovInd} = intersect(outPlatRings.inner, rings.outer);
        if length(initPos{outerMovInd}) ~= 3
            error('clearly a problem!')
        end
        removeInd = false(length(initPos{outerMovInd})); % should be 3 possible 
        % start platforms; remove the one that is adjacent to the other 2
        for i = 1:length(initPos{outerMovInd})
            plat1 = initPos{outerMovInd}(i);
            plats2and3 = setdiff(initPos{outerMovInd}, ...
                initPos{outerMovInd}(i));            
            adjFlag1 = checkAdjacent(plat1, plats2and3(1), platformMap);
            adjFlag2 = checkAdjacent(plat1, plats2and3(2), platformMap);
            
            if adjFlag1 && adjFlag2
                removeInd(i) = true;
            end
        end
        initPos{outerMovInd}(removeInd) = [];
    end
    movFirst = outerMovInd;
end
end

       







