function straightPath = getStraigthPath(plat1, plat2, platformMap)
%
%%
inInd1 = find(platformMap == plat1);
[in1Row, in1Col] = ind2sub(size(platformMap), inInd1);

inInd2 = find(platformMap == plat2);
[in2Row, in2Col] = ind2sub(size(platformMap), inInd2);

rowDiff = in2Row - in1Row;
colDiff = in2Col - in1Col;

if colDiff == 0 % move along column
    nPlats = 1 + abs(rowDiff)/2;
    
    if rowDiff >= 0
        rowInd = (in1Row:2:in2Row)';
    else
        rowInd = (in1Row:-2:in2Row)';
    end
    
    colInd = repmat(in1Col, length(rowInd),1);
    
else % move along diagonal
    nPlats = 1 + abs(rowDiff);
    
    if rowDiff >= 0
        rowInd = (in1Row:1:in2Row)';
    else
        rowInd = (in1Row:-1:in2Row)';
    end
    
    if colDiff >= 0
        colInd = (in1Col:1:in2Col)';
    else
        colInd = (in1Col:-1:in2Col)';
    end    
end
    
mapInd = sub2ind(size(platformMap), rowInd, colInd);
straightPath = platformMap(mapInd);