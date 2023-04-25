% drive2allPlats
% for acquiring images of platform locations
% initial robot platform positions start at platform 41, initial position
% is 120 degree.
%%
load robotInit.mat platformMap timePerGap timePerLine
load robotInit_Five.mat
% load platformCoordinates.mat
nRobots = 5;

%% connect to robots

if ~exist('robotCh', 'var') % unless already connected
    robotCh = cell(1,nRobots);
end
robotCh = connect2Robots(robot, robotCh);
%% set up camera
vid = videoinput('winvideo',1,'RGB24_2448x2048');
% vid = videoinput('winvideo',2,'RGB24_2448x2048');

%%
startPlatform = 41;
startInd = find(platformMap(:) == startPlatform);
[startRow, startCol] = ind2sub(size(platformMap), startInd); 
currRows = startRow:2:startRow + 2*(nRobots-1);

stopPlat = 217;
stopInd = find(platformMap(:) == stopPlat);
[stopRow, stopCol] = ind2sub(size(platformMap), stopInd); 

nCols = stopCol - startCol + 1;
nRows = stopRow - startRow + 1;

currCol = startCol;

startingPlats = platformMap(currRows, currCol); 

for r = 1:5
    robot(r).pos = startingPlats(r);
end

counter = 1;
while true

    currentPlats = platformMap(currRows, currCol);    
    
    if counter ~= 1 % move robots
        
        for r = 1:5
            if counter ~= nCols+1
                pathsFinal{r} = [prevPlats(r), currentPlats(r)];
            else
                pathsFinal{r} = platformMap(prevRows(r):2:currRows(r), currCol);
            end
        end
        
        robotInput = ...
            constructRobotPaths_Simple(robot(1), pathsFinal{r}, platformMap); 

        [duration, robotInput] = ...
            getPathTimings_Simple(robotInput, timePerGap, timePerLine);

        if counter <= (nCols + 1) && mod(counter, 2) ~= 0
            robotOrder = 1:5;
            
        elseif counter <= (nCols + 1) && mod(counter, 2) == 0
            robotOrder = 5:-1:1;
            
        elseif counter > (nCols + 1) && mod(counter, 2) ~= 0
            robotOrder = 5:-1:1;
        else
            robotOrder = 1:5;
        end
        
        for r = 1:length(robotOrder)
            if r == length(robotOrder)
                tic
            end

            r2 = robotOrder(r);
            robot(r2) = sendSSHcommands_4AllPlats(robot(r2), robotInput, pathsFinal{r2}, ...
                duration, robotCh{r2});
            pause(0.5)
        end

        while true
            elapsedTime = toc;
            pause(0.1)
            if elapsedTime > duration
                break
            end
        end
    end
    %% take image
    % optional cropping, only useful if we already know roughly where the
    % platforms are. We can always crop later
%     cropNums  = getCropNums(currentPlats, platCoor);
%     writematrix(cropNums, 'cropNums.csv')
    % 

%     imageFile = fopen('takeImage.txt','w');
%     fprintf(imageFile, 'platforms_%u_%u_%u_%u_%u\n', currentPlats);
%     fclose(imageFile);
% 
    imageFile = sprintf('platforms_%u_%u_%u_%u_%u', currentPlats);
    img = getsnapshot(vid);

    save([imageFile '.mat'], 'img')    
    imwrite(img,[imageFile '.jpeg'])
    pause(0.1)
   
    %%
    prevPlats = currentPlats;
    prevRows = currRows;

    if counter < nCols
        currCol = currCol+1;
        if mod(counter, 2) == 1
            currRows = currRows + 1;
        else
            currRows = currRows - 1;
        end
                    
    elseif counter == nCols
        currRows = currRows + nRobots*2;

    elseif counter == 2*nCols
        break

    else
        currCol = currCol-1;
        if mod(counter, 2) == 1
            currRows = currRows - 1;
        else
            currRows = currRows + 1;
        end
    end
   
    counter = counter + 1;
end