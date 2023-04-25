function [nextPlats, possPlats] = getNextPlats(statRobot, nonStatRobot, platformMap, excludedPlats)
% choose next platforms
%%
[rings, ~] = getRings(statRobot, platformMap);
possPlats = rings.inner;
[~, iP, ~] = intersect(possPlats, excludedPlats);
possPlats(iP) = [];

while true
    nextPlats = randsample(possPlats, 2);
    
    if ~isempty(find(~ismember(nextPlats, nonStatRobot), 1)) || length(possPlats) == 2
        break
    end
end
