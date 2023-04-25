function interPlatDist = getDistance(plat1, plat2, platformMap)
%% function to calculate distance between 2 platforms
plat1ind = find(platformMap(:) == plat1);
[plat1Row, plat1Col] = ind2sub(size(platformMap), plat1ind);

plat2ind = find(platformMap(:) == plat2);
[plat2Row, plat2Col] = ind2sub(size(platformMap), plat2ind);

rowDiff = abs(plat1Row - plat2Row);
colDiff = abs(plat1Col - plat2Col);

if rowDiff <= colDiff
    interPlatDist = colDiff;
    
else
    rowColDiff = rowDiff - colDiff;
    interPlatDist = colDiff + rowColDiff/2;
end
end
