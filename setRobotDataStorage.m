function [fileName, choices] = setRobotDataStorage(goalPlatform)
% this stores all the trial results for the robot honeycomb maze
%%
presentDir = pwd;
folderInd = regexp(presentDir, '\') + 1;

% get bottom two folders, since we might already be in \date\goal
if length(folderInd) == 1
    folderTop = presentDir(1:folderInd-2);
    folderBottom = presentDir(folderInd:end);
else
    folderTop = presentDir(folderInd(end-1):folderInd(end)-2);
    folderBottom = presentDir(folderInd(end):end);
end

currDate = datestr(date, 'dd-mm-yyyy');
if ~strcmp(folderTop, currDate) && ~strcmp(folderBottom, currDate)
    if ~exist(currDate, 'dir')
        mkdir(currDate)
    end
    cd(currDate)  
end

if ~strcmp(folderBottom, ['goal_' num2str(goalPlatform)])
    if ~exist(['goal_' num2str(goalPlatform)], 'dir')
        mkdir(['goal_' num2str(goalPlatform)])  
    end
    cd(['goal_' num2str(goalPlatform)])  
end

currTime = clock; 
currTime_year = num2str(currTime(1));
currTime_month = num2str(currTime(2));
if length(currTime_month) == 1
    currTime_month = ['0' currTime_month];
end
currTime_day = num2str(currTime(3));
if length(currTime_day) == 1
    currTime_day = ['0' currTime_day];
end
currTime_hour = num2str(currTime(4));
currTime_min = num2str(currTime(5));
if length(currTime_min) == 1
    currTime_min = ['0' currTime_min];
end

currTimeStr = [currTime_year '_' currTime_month '_' currTime_day '_' currTime_hour 'h' currTime_min 'm'];
fileName = ['trial_' currTimeStr];

matFiles = dir('*.mat');
matFiles = {matFiles(:).name};
if ~isempty(find(strcmp(matFiles, 'choices.mat'), 1))
    load choices.mat choices
end

choices.(fileName) = [];

choices.(fileName)(1).time = [];
choices.(fileName)(1).startPlatform = [];
choices.(fileName)(1).chosenPlatform = [];
choices.(fileName)(1).unchosenPlatform = [];