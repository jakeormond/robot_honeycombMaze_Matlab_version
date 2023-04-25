% saveRobotInit_Five
% set the intitial positions, ip addresses, etc. of the robots

%% set initial robot directions and ip addresses

% robot initial positions and orientations
robot(1).dir = 120;
robot(1).ip = [192, 168 ,0, 107];

robot(2).dir = 120; % so that they can be moved into position once program is started.
robot(2).ip = [192, 168 ,0, 103];

robot(3).dir = 120;
robot(3).ip = [192, 168 ,0, 106];

robot(4).dir = 120;
robot(4).ip = [192, 168 ,0, 102];

robot(5).dir = 120; % so that they can be moved into position once program is started.
robot(5).ip = [192, 168 ,0, 105];

save robotInit_Five.mat robot 

