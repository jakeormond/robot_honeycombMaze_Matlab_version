function direction = getDirection(currPlat, nextPlat, platformMap)
%
%%
inInd1 = find(platformMap == currPlat);
[in1Row, in1Col] = ind2sub(size(platformMap), inInd1);

inInd2 = find(platformMap == nextPlat);
[in2Row, in2Col] = ind2sub(size(platformMap), inInd2);

rowDiff = in2Row - in1Row;
colDiff = in2Col - in1Col;

direction = NaN;
if rowDiff < 0 
    if colDiff == 0
        direction = 0;
        
    elseif rowDiff/colDiff == 1
        direction = 300;
        
    elseif rowDiff/colDiff == -1
        direction = 60;
    end
    
elseif rowDiff > 0
    if colDiff == 0
        direction = 180;    
    
    elseif rowDiff/colDiff == 1
        direction = 120;
        
    elseif rowDiff/colDiff == -1
        direction = 240;
    end
 end
        
    
    