function [minLengths, maxLengths] = checkAllPaths(robot, initialPos, ...
    paths, movFirst, platformMap)
% returns min and max lengths of all compatible pairs. 
% NaN indicates that paths in a given pair are not compatible
%%
compMatrix = false(2,length(initialPos{1}),2,length(initialPos{2}),2);
% staggerMatrix = zeros(2,3,2,3,2);
maxLengths = NaN(2,length(initialPos{1}),2,length(initialPos{2}),2);
minLengths = NaN(2,length(initialPos{1}),2,length(initialPos{2}),2);
pathsTemp = cell(1,2);

statRobot = find(strcmp({robot(:).movState}, 'stationary'));
nonStatRobots = find(~ismember(1:3, statRobot));

for p1 = 1:2 % robot1's platform, which determines robot2's platform
    p2 = find(~ismember([1 2], p1));

    for i1 = 1:length(initialPos{1}) % robot1 start platform
        for d1 = 1:2 % robot1's direction
            pathsTemp{1} = [robot(nonStatRobots(1)).pos; ...
                paths{1, p1, i1, d1}];

            for i2 = 1:length(initialPos{2})
                for d2 = 1:2 % robot2's direction

                    sameDir = d2 == d1;
                    if movFirst > 0
                        if sameDir
                            continue % robots must move in opposite directions in this specific condition
                        end
                    end

                    pathsTemp{2} = [robot(nonStatRobots(2)).pos; ...
                        paths{2, p2, i2, d2}];

                    compFlag = checkPathCompatibility(pathsTemp, sameDir, ...
                        platformMap);

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
end

%%
function compFlag = checkPathCompatibility(pathsTemp, sameDir, platformMap)
% this is specific to the circle platforms, where they only travel around
% the stationary platform in the middle ring. As such, if they are moving
% in the same direction, the only stipulation is that one robot can't
% leapfrog the other. If they are moving in opposite directions, their
% paths can't include any identical positions. 

compFlag = false;

%% regardless of direction, if one path is only 2 positions long (meaning 
% the robot moves from its current position directly to its final position)
% then the other robot can not move through or adjacent to its initial
% position
if min(cellfun(@length, pathsTemp)) == 2
    shortPath = find(cellfun(@length, pathsTemp) == 2);
    longPath = find(cellfun(@length, pathsTemp) > 2);
    
    if isempty(longPath) % only occurs during trial start
        compFlag = true;
        return
    end
    
    for p = 2:length(pathsTemp{longPath})
        if pathsTemp{longPath}(p) == pathsTemp{shortPath}(1) || ...
                checkAdjacent(pathsTemp{longPath}(p), ...
                pathsTemp{shortPath}(1), platformMap)
            
            return
        end
    end
end

% regardless of direction, robots can't be initially adjacent and adjacent
% in the second position unless they are moving in the same direction.
if checkAdjacent(pathsTemp{1}(1), pathsTemp{2}(1), platformMap) && ...
        checkAdjacent(pathsTemp{1}(2), pathsTemp{2}(2), platformMap)
    
    if getDirection(pathsTemp{1}(1), pathsTemp{1}(2), platformMap) ~= ... 
        getDirection(pathsTemp{2}(1), pathsTemp{2}(2), platformMap)
        
        return
    end
end


if ~sameDir % opposite directions, so can't contain any identical positions
    % and can't be adjacent in first 2 positions
    if checkAdjacent(pathsTemp{1}(1), pathsTemp{2}(1), platformMap) && ...
            (checkAdjacent(pathsTemp{1}(1), pathsTemp{2}(2), platformMap) || ...
            pathsTemp{1}(1) == pathsTemp{2}(2)) && ...
            (checkAdjacent(pathsTemp{1}(2), pathsTemp{2}(1), platformMap) || ...
            pathsTemp{2}(1) == pathsTemp{1}(2))
    
    elseif isempty(intersect(pathsTemp{1}(2:end), pathsTemp{2}(2:end)))
        compFlag = true;
    end
    
    return
end

% if same direction, if paths intersect they will still be compatible
% if one path contains the first position of the other path, while
% this other path contains the final position of the first path;
% furthermore, the first path must have a unique start, and the second
% path must have a unique end. Lastly, the first 2 positions of each robot
% can't both be adjacent, as they will likely bump into each other. 

% can't have first 2 positions being adjacent
if length(pathsTemp{1}) > 2 % this should only not be the case during the trial start when moving platforms are already away from stationary platform
    if checkAdjacent(pathsTemp{1}(2), pathsTemp{2}(2), platformMap)
        if length(pathsTemp{2}) == 2 || ...
                checkAdjacent(pathsTemp{1}(3), pathsTemp{2}(3), platformMap)
            return
        end
    end
end

if pathsTemp{1}(2) == pathsTemp{2}(2) % can't move to the same initial positions
    return
end
    
if pathsTemp{1}(end-1) == pathsTemp{2}(end-1) % can't move to the same final positions
    return
end
    
% for paths in same direction
if ~isempty(intersect(pathsTemp{1}(2:end), pathsTemp{2}(2))) % path 1 contains start of path 2
    if ~isempty(intersect(pathsTemp{1}(end), pathsTemp{2}(2:end))) % path 2 contains end of path 1
        compFlag = true;                                            % then they are compatible
        return
    else        % path 2 doesn't contain end of path 1 (this means path 2 is contained within path 1)
        return  % then not compatible
    end
    
elseif ~isempty(intersect(pathsTemp{1}(2), pathsTemp{2}(2:end))) % path 2 contains start of path 1
    if ~isempty(intersect(pathsTemp{1}(2:end), pathsTemp{2}(end))) % path 1 contains end of path 2
        compFlag = true;                                            % then they are compatible
        return
    else        % path 1 doesn't contain end of path 2 (this means path 1 is contained within path 2)
        return  % then not compatible
    end
    
else  % 
    compFlag = true;
    return   
end
        
end