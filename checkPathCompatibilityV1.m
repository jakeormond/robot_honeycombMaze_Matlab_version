function [compFlag, stagger] = checkPathCompatibility(pathsTemp, ...
    movFirst, platformMap)
%%
if movFirst == 0 % robots move away from stationary platform at the same time
    stagger = 0;
    pathFirst = pathsTemp{1};
    pathSecond = pathsTemp{2};
    
    minLength = min(length(pathFirst), length(pathSecond));
    
    for p = 1:minLength
        if checkAdjacent(pathFirst(p), pathSecond(p), platformMap) ...
                || pathFirst(p) == pathSecond(p)
            
            compFlag = false;
            break
        end
        compFlag = true;
    end
    
else    
    stagger = NaN;
    if movFirst == 1 || movFirst == 2 % check if they are staggered
        movSecond = ~ismember([1,2], movFirst);
        
        if length(pathsTemp{movFirst}) == 1 || ...
                isempty(intersect(pathsTemp{1}, pathsTemp{2}))
            
            compFlag = true;
            stagger = 1;
            
        else
            for p = 1:length(pathsTemp{movFirst})-1
                pathFirst = pathsTemp{movFirst}(p+1:end);
                pathSecond = pathsTemp{movSecond};
                if length(pathSecond) > length(pathFirst)
                    lenDiff = length(pathSecond) - length(pathFirst);
                    pathFirst = [pathFirst; repmat(pathFirst(end), lenDiff, 1)];
                    
                elseif length(pathFirst) > length(pathSecond)
                    lenDiff = length(pathFirst) - length(pathSecond);
                    pathSecond = [pathSecond; repmat(pathSecond(end), lenDiff, 1)];
                end
                
                minLength = min(length(pathFirst), length(pathSecond));
                
                adjFlag = false(minLength, 1);
                for p2 = 1:minLength
                    if checkAdjacent(pathFirst(p2), pathSecond(p2), ...
                            platformMap) || pathFirst(p2) == pathSecond(p2)
                        
                        adjFlag(p2) = true;
                    end
                end
                
                if isempty(find(adjFlag, 1))
                    stagger = p;
                    compFlag = true;
                    break
                end
            end
        end
        
        if isnan(stagger)
            compFlag = false;
            
          % CODE BELOW DOESN'T WORK IN SITUATIONS WHERE OTHER ROBOT IS GOING TO HAVE TO WAIT  
%         else % still need to check whether intial part of the path is
%             % compatible with other robots initial position
%             pathFirst = pathsTemp{movFirst}(1:stagger-1);
%             pathSecond = pathsTemp{movSecond}(1);
%             
%             for p = 1:length(pathFirst)
%                 if checkAdjacent(pathFirst(p), pathSecond, ...
%                         platformMap) || pathFirst(p) == pathSecond
%                     stagger = NaN;
%                     compFlag = false;
%                 end
%             end
        end
    end
end






