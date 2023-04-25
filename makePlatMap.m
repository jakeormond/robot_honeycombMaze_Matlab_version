function platformMap = makePlatMap(nRows, nCols)
% make the map for the robot maze
% 28/9/2021: need to modify map, first row is 9 long second is 10. 29 rows
% in total
%%
% NOTE: the maze doesn't actually have columns and rows because 
% it's arranged on 3 axes, but this is a useful way to build it
if length(nCols) == 2
    nColsForMatrix = sum(nCols);
    platformMap = NaN(nRows,nColsForMatrix); % the map
    
    lastPlat = 0;
    for i = 1:nRows
        if i > 1
            lastPlat = platforms(end);
        end
        
        if mod(i,2) == 1
            platforms = lastPlat + (1:nCols(1));
            
            if nCols(1) < nCols(2)                      
                platformMap(i, 2:2:end) = platforms;
            else
                platformMap(i, 1:2:end) = platforms;
            end
            
        else
            platforms = lastPlat + (1:nCols(2));
            
            if nCols(1) < nCols(2)
                platformMap(i, 1:2:end) = platforms;
            else
                platformMap(i, 2:2:end) = platforms;
            end
        end
    end
    
    
else
    platformMap = NaN(nRows,nCols*2); % the map
    
    for i = 1:nRows
        if mod(i,2) == 1
            platformMap(i, 2:2:end) = nCols*(i-1) + (1:nCols);
        else
            platformMap(i, 1:2:end) = nCols*(i-1) + (1:nCols);
        end
    end    
end

