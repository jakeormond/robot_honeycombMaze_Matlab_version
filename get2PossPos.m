function possPos = get2PossPos(pos, direction, platformMap)
% get the next 2 possible positions for a robot given it's current position
% and orientation. It assumes the robot can't turn due to being adjacent to
% another robot.
% possPos has 2 values: first, the position in front, second, the position
% behind
%%
posInd = find(platformMap == pos);
[posRow, posCol] = ind2sub(size(platformMap), posInd);

if direction == 0
    possPos(1) = platformMap(posRow-2, posCol);
    possPos(2) = platformMap(posRow+2, posCol);
    
elseif direction == 60
    possPos(1) = platformMap(posRow-1, posCol+1);
    possPos(2) = platformMap(posRow+1, posCol-1);
    
elseif direction == 120
    possPos(1) = platformMap(posRow+1, posCol+1);
    possPos(2) = platformMap(posRow-1, posCol-1);
    
elseif direction == 180
    possPos(1) = platformMap(posRow+2, posCol);
    possPos(2) = platformMap(posRow-2, posCol);
    
elseif direction == 240
    possPos(1) = platformMap(posRow+1, posCol-1);
    possPos(2) = platformMap(posRow-1, posCol+1);
    
elseif direction == 300
    possPos(1) = platformMap(posRow-1, posCol-1);
    possPos(2) = platformMap(posRow+1, posCol+1);
end

