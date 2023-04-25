function compFlag = checkPathCompatibilityV2(pathsTemp, platformMap, d1, d2)
% this is specific to the circle platforms, where they only travel around
% the stationary platform in the middle ring. As such, if they are moving
% in the same direction, the only stipulation is that one robot can't
% leapfrog the other. If they are moving in opposite directions, their
% paths can't include any identical positions. 
%%
compFlag = false;
% if the 2 paths have no overlapping positions, then they are compatible
% if isempty(intersect(pathsTemp{1}, pathsTemp{2}))
%     compFlag = true;
%     return
% end

% if d1 ~= d2 
%     return
% end

% if same direction, if paths intersect they will still be compatible
% if one path contains the first position of the other path, while
% this other path contains the final position of the first path;
% furthermore, the first path must have a unique start, and the second
% path must have a unique end. Lastly, the first 2 positions of each robot
% can't both be adjacent, as they will likely bump into each other. 
firstPosRings = getRings(pathsTemp{1}(1), platformMap);
if length(pathsTemp{1}) > 1 % this should only not be the case during the trial start when moving platforms are already away from stationary platform
    secondPosRings = getRings(pathsTemp{1}(2), platformMap);
    if ~isempty(intersect(firstPosRings.inner, pathsTemp{2}(1))) && ...
            (length(pathsTemp{2}) == 1 || ...
            ~isempty(intersect(secondPosRings.inner, pathsTemp{2}(2))))
        return
    end
end

if pathsTemp{1}(1) == pathsTemp{2}(1)
    return
end
    
if pathsTemp{1}(end) == pathsTemp{2}(end)
    return
end
    
if ~isempty(intersect(pathsTemp{1}, pathsTemp{2}(1)))
    if ~isempty(intersect(pathsTemp{1}(end), pathsTemp{2}))
        compFlag = true;
        return
    else
        return
    end
    
elseif ~isempty(intersect(pathsTemp{1}(1), pathsTemp{2}))
    if ~isempty(intersect(pathsTemp{1}, pathsTemp{2}(end)))
        compFlag = true;
        return
    else
        return
    end
    
else
    compFlag = true;
    return   
end
        
end