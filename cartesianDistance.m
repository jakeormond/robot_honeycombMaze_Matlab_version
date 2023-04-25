function distance = cartesianDistance(platform1, platform2, platformMap)
%
%%
% distance between platform rows is 1 row = 1 a.u.
% distance between columns is = 1 a.u. / tan(30deg) = 1.7321 a.u.
rowDist = 1;
colDist = rowDist / tan(deg2rad(30));

linInd1 = find(platformMap == platform1);
[inRow1, inCol1] = ind2sub(size(platformMap), linInd1);

linInd2 = find(platformMap == platform2);
[inRow2, inCol2] = ind2sub(size(platformMap), linInd2);

rowDiff = inRow1 - inRow2;
colDiff = inCol1 - inCol2;

distance = sqrt((rowDiff * rowDist)^2 + (colDiff * colDist)^2);