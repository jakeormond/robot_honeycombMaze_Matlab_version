function adjFlag = checkAdjacent(plat1, plat2, platformMap)
%
%%
plat1ring = getRings(plat1, platformMap);
if ismember(plat2, plat1ring.inner)
    adjFlag = true;
else
    adjFlag = false;
end