function [minLengths, maxLengths] checkAllPaths(robot, initialPos, paths)
% returns min and max lengths of all compatible pairs. 
%%
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


















%%
function compFlag = checkPathCompatibility(pathsTemp, platformMap, d1, d2)
% this is specific to the circle platforms, where they only travel around
% the stationary platform in the middle ring. As such, if they are moving
% in the same direction, the only stipulation is that one robot can't
% leapfrog the other. If they are moving in opposite directions, their
% paths can't include any identical positions. 

compFlag = false;

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