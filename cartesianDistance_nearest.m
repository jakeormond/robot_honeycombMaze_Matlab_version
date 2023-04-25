function distance = cartesianDistance_nearest(platform1, platform2, platformMap)
%
%%
% distance between platform rows is 1 row = 1 a.u.
% distance between columns is = 1 a.u. / tan(30deg) = 1.7321 a.u.
rowDist = 1;
colDist = rowDist / tan(deg2rad(30));

linInd1 = find(platformMap == platform1);
[inRow1, inCol1] = ind2sub(size(platformMap), linInd1);

distance = NaN(length(platform2), 1);
for p = 1:length(platform2)
    linInd2 = find(platformMap == platform2(p));
    [inRow2, inCol2] = ind2sub(size(platformMap), linInd2);

    rowDiff = inRow1 - inRow2;
    colDiff = inCol1 - inCol2;

    distance(p) = sqrt((rowDiff * rowDist)^2 + (colDiff * colDist)^2);
end
distance = min(distance);