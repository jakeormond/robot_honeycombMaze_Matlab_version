function sameDirPath = moveInSameDirection(inPosition, nextPosition, ...
    nNodes, platformMap)
% calculate a path in a single direction given first 2 positions and number
% of nodes. 
%%
inInd = find(platformMap == inPosition);
[inRow, inCol] = ind2sub(size(platformMap), inInd);


nextInd = find(platformMap == nextPosition);
[nextRow, nextCol] = ind2sub(size(platformMap), nextInd);

rowDiff =  nextRow-inRow;
colDiff = nextCol-inCol;

sameDirPath = nextPosition;

for n = 1:nNodes
    sameDirPath = [sameDirPath; ...
        platformMap(nextRow+rowDiff*n, nextCol+colDiff)]; 
end

