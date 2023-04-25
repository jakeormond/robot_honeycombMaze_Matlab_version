function [rings, vertices] = getRings(plat, platformMap)
%% function to get inner and outer rings
ringRelInd{1} = [-2, 0; -1, 1; 1, 1; 2, 0; 1, -1; -1, -1];

ringRelInd{2} = [-4, 0; -3, 1; -2, 2; 0, 2; 2, 2; 3, 1; 4, 0; 3, -1; ...
    2, -2; 0, -2; -2, -2; -3, -1];
vertRelInd{2} = [-4, 0; -2, 2; 2, 2; 4, 0; 2, -2; -2, -2];

ringRelInd{3} = [-6, 0; -5, 1; -4, 2; -3, 3; -1, 3; 1, 3; 3, 3; 4, 2; ...
    5, 1; 6, 0; 5, -1; 4, -2; 3, -3; 1, -3; -1, -3; -3, -3; -4, -2; -5, -1];
vertRelInd{3} = [-6, 0; -3, 3; 3, 3; 6, 0; 3, -3; -3, -3];
%%
statInd = find(platformMap == plat);
[statRow, statCol] = ind2sub(size(platformMap), statInd);
%%
for i = 1:length(ringRelInd)
    ringSub = [statRow + ringRelInd{i}(:,1), ...
        statCol + ringRelInd{i}(:,2)];
    
    % remove positions that are off the map
    removeInd = ringSub(:,1) < 1 | ...
        ringSub(:,1) > size(platformMap, 1) | ...
        ringSub(:,2) < 1 | ...
        ringSub(:,2) > size(platformMap, 2);
    ringSub(removeInd, :) = [];
    
    ringInd = sub2ind(size(platformMap), ringSub(:,1), ringSub(:,2));
    ring = platformMap(ringInd);
    
    if i ~= 1
        vertSub = [statRow + vertRelInd{i}(:,1), ...
            statCol + vertRelInd{i}(:,2)];
        % remove positions that are off the map
        removeInd = vertSub(:,1) < 1 | ...
            vertSub(:,1) > size(platformMap, 1) | ...
            vertSub(:,2) < 1 | ...
            vertSub(:,2) > size(platformMap, 2);
        vertSub(removeInd, :) = [];
        
        vertInd = sub2ind(size(platformMap), vertSub(:,1), vertSub(:,2));
        vert = platformMap(vertInd);
    end
    
    if i == 1
        rings.inner = ring;
    elseif i == 2
        rings.middle = ring;
        vertices.middle = vert;
    else
        rings.outer = ring;
        vertices.outer = vert;
    end
end
% %%
% middleRingSub = [statRow + middleRingRelInd(:,1), ...
%     statCol + middleRingRelInd(:,2)];
% % remove positions that are off the map
% removeInd = middleRingSub(:,1) < 1 | ...
%     middleRingSub(:,1) > size(platformMap, 1) | ...
%     middleRingSub(:,2) < 1 | ...
%     middleRingSub(:,2) > size(platformMap, 2);
% middleRingSub(removeInd, :) = [];
% 
% middleRingInd = sub2ind(size(platformMap), middleRingSub(:,1), middleRingSub(:,2));
% middleRing = platformMap(middleRingInd);
% 
% %%
% middleRingSub = [statRow + middleRingRelInd(:,1), ...
%     statCol + middleRingRelInd(:,2)];
% % remove positions that are off the map
% removeInd = middleRingSub(:,1) < 1 | ...
%     middleRingSub(:,1) > size(platformMap, 1) | ...
%     middleRingSub(:,2) < 1 | ...
%     middleRingSub(:,2) > size(platformMap, 2);
% middleRingSub(removeInd, :) = [];
% 
% middleRingInd = sub2ind(size(platformMap), middleRingSub(:,1), middleRingSub(:,2));
% middleRing = platformMap(middleRingInd);
% 
% 
% 
% 
% 

end
