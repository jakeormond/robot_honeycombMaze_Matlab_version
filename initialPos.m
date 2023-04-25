function possPos = initialPos(movRobot, statRobot, platformMap)
% get the next 2 possible positions for a robot given it's current position
% and orientation. It assumes the robot can't turn due to being adjacent to
% another robot.
% possPos has 2 values: first, the position in front, second, the position
% behind
%%
for r = 1:length(robot)
    posInd = find(platformMap == robot(r).pos);
    [posRow, posCol] = ind2sub(size(platformMap), posInd);
    
    if robot(r).dir == 0
        possPosTemp(1) = platformMap(posRow-2, posCol);
        possPosTemp(2) = platformMap(posRow+2, posCol);
        
    elseif robot(r).dir == 60
        possPosTemp(1) = platformMap(posRow-1, posCol+1);
        possPosTemp(2) = platformMap(posRow+1, posCol-1);
        
    elseif robot(r).dir == 120
        possPosTemp(1) = platformMap(posRow+1, posCol+1);
        possPosTemp(2) = platformMap(posRow-1, posCol-1);
        
    elseif robot(r).dir == 180
        possPosTemp(1) = platformMap(posRow+2, posCol);
        possPosTemp(2) = platformMap(posRow-2, posCol);
        
    elseif robot(r).dir == 240
        possPosTemp(1) = platformMap(posRow+1, posCol-1);
        possPosTemp(2) = platformMap(posRow-1, posCol+1);
        
    elseif robot(r).dir == 300
        possPosTemp(1) = platformMap(posRow-1, posCol-1);
        possPosTemp(2) = platformMap(posRow+1, posCol+1);
    end
    
    if length(robot) == 1
        possPos = possPosTemp;
    else
        possPos{r,1} = possPosTemp;
    end
end