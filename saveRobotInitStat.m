% saveRobotInitStat
% set the intitial positions, ip addresses, etc. of the robots
%% make the platform map
nRows = 26; nCols = [9, 10];
platformMap = makePlatMap(nRows, nCols);
restrictedMap = platformMap;

%% make list of platform positions that can't be used for final positions.
% These positions are excluded because it's impossible to navigate robots
% close to walls
excludedPlatsCell{1} = platformMap(1:4, :); restrictedMap(1:4, :) = [];
excludedPlatsCell{2} = platformMap(:, 1:5); restrictedMap(:, 1:5) = [];
excludedPlatsCell{3} = platformMap(end-3:end, :); restrictedMap(end-3:end, :) = [];
excludedPlatsCell{4} = platformMap(:, end-2:end); restrictedMap(:, end-2:end) = [];

for i = 1:4
    excludedPlatsCell{i} = excludedPlatsCell{i}(:);
end
excludedPlats = unique(vertcat(excludedPlatsCell{:}));
excludedPlats(isnan(excludedPlats)) = [];

%% 6 possible directions = N, NE, SE, S, SW, SE
direction.N = 0; direction.NE = 60; direction.SE = 120; direction.S = 180;
direction.SW = 240; direction.NW = 300;

%% set initial robot directions and ip addresses

% robot initial positions and orientations
robot(1).dir = 180;
robot(1).movState = 'stationary';
robot(1).ip = [192, 168 ,0, 105];

robot(2).dir = 0; % so that they can be moved into position once program is started.
robot(2).movState = 'moving';
robot(2).ip = [192, 168 ,0, 110];

robot(3).dir = 0;
robot(3).movState = 'moving';
robot(3).ip = [192, 168 ,0, 107];

% robot2(1).dir = 180;
% robot2(1).movState = 'stationary';
% robot2(1).ip = [192, 168 ,0, 102];
% 
% robot2(2).dir = 0; % so that they can be moved into position once program is started.
% robot2(2).movState = 'moving';
% robot2(2).ip = [192, 168 ,0, 104];
% 
% robot2(3).dir = 0;
% robot2(3).movState = 'moving';
% robot2(3).ip = [192, 168 ,0, 105];

%% set variables
timePerGap = 3.5;
timePerLine = 3.5;

duraSpin = 5;

save robotInit.mat platformMap direction robot ...
    timePerGap timePerLine duraSpin excludedPlats

% robot = robot2;
% 
% save robotInit2.mat platformMap direction robot ...
%     timePerGap timePerLine duraSpin excludedPlats
