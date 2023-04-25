function platform = getDecision(currentPlat, possiblePlats, platCoor, cropNums)
%
%% get platform coordinates
allPlats = [currentPlat; possiblePlats];
coor = NaN(length(allPlats),3); % each row is a platform, arranged [x, y, radius];

for p = 1:length(allPlats)
    coor(p, 1:2) = platCoor(allPlats(p)).Centre;
    coor(p, 3) = platCoor(allPlats(p)).Radius;
end

currPlatCoor(1:2) = platCoor(currentPlat).Centre;
currPlatCoor(3) = platCoor(currentPlat).Radius;

%%
% Specify a LocalHost (host name or IP address) if known
u = udpport('LocalHost', '0.0.0.0', 'LocalPort', 8000, 'Timeout', 100);
fopen(u);
flush(u)

b = 256;

nTrackingPts = 3;
nPlats = length(allPlats);
minPlatTime = 4;
counter = 1;
counter2 = 0;

while true
    counter2 = counter2+1;
    if counter == 1
        platformPosition = [];
        elapsedTime = [];
        totalDist_avgCoor = [];
    end

    tic
    packetData = read(u,40)';
    flush(u)

    ratCoor = NaN(nTrackingPts,2);

    ratCoor(1,1) = packetData(19)*b + packetData(20); % x coordinate
    ratCoor(1,2) = packetData(23)*b + packetData(24); % y coordinate

    ratCoor(2,1) = packetData(27)*b + packetData(28);
    ratCoor(2,2) = packetData(31)*b + packetData(32);

    ratCoor(3,1) = packetData(35)*b + packetData(36);
    ratCoor(3,2) = packetData(39)*b + packetData(40);

    ratCoor(:,1) = ratCoor(:,1) + cropNums(1);
    ratCoor(:,2) = ratCoor(:,2) + cropNums(2);

%     ratCoor(1,1) = 254; % x coordinate
%     ratCoor(1,2) = 491; % y coordinate
% 
%     ratCoor(2,1) = 260;
%     ratCoor(2,2) = 497;
% 
%     ratCoor(3,1) = 276;
%     ratCoor(3,2) = 513;

%     ratCoor(:,1) = ratCoor(:,1) + cropNums(1);
%     ratCoor(:,2) = ratCoor(:,2) + cropNums(2);

    %  get distance of each mark to each platform, and determine the
    %  identity of the platform occupied
    totalDist = NaN(nTrackingPts, nPlats);
    ratPlatTemp = NaN(nTrackingPts,1);    

    for r = 1:3 % these are the tracking positions on the rat
        for p = 1:length(allPlats) % the possible platform locations
            xDist = ratCoor(r,1) - coor(p,1);
            yDist = ratCoor(r,2) - coor(p,2);
            totalDist(r,p) = sqrt(xDist^2 + yDist^2);
            
%             if totalDist <= coor(p,3) % radius of the platform
%                 ratPlatTemp(r) = p;
%                 break
%             end
        end
        [~, platInd] = min(totalDist(r, :));
        ratPlatTemp(r) = allPlats(platInd);

    end

    % determine if he has selected one the choice platforms
    if ismember(ratPlatTemp(1), possiblePlats) && ...
            length(unique(ratPlatTemp)) == 1 % then yes, he has made a selection

        platformPosition(counter, 1) = ratPlatTemp(1);

    else
        counter = 1;
        flush(u)
        continue % no decision made 
    end

    % determine if selected platform is same as from previous loop. If
    % not, then restart the observation
   
    if counter > 1 && platformPosition(counter) ~= platformPosition(counter - 1)
        platformPosition = platformPosition(counter);
        elapsedTime = toc;
        counter = 1;
        counter = counter + 1;
        flush(u)
        continue
    end

    % even though he has selected a platform, we still need to determine if
    % he is moving towards the other platforms since he could potentially
    % move to another platform before the robots move away.
    % First step is get his distance to each platform
    xAvg = mean(ratCoor(1,1), ratCoor(3,1));
    yAvg = mean(ratCoor(1,2), ratCoor(3,2));

    totalDist_avgCoor(counter, :) = NaN(1, nPlats);
    for p = 1:nPlats
        xDist = xAvg - coor(p,1);
        yDist = yAvg - coor(p,2);
        totalDist_avgCoor(counter,p) = sqrt(xDist^2 + yDist^2);
    end

    % Second step is to deterimine if he has occupied the platform for
    % enough time, otherwise we can just step to the next loop
    if isempty(elapsedTime) || sum(elapsedTime) < minPlatTime
        elapsedTime(counter, 1) = toc;
        counter = counter + 1;  
        flush(u)
        continue
    end

    % Third step is determine if he's moving towards the other platforms
    otherInd = find(allPlats ~= platformPosition(counter));
    currInd = find(allPlats == platformPosition(counter));
    smoothCurr = smoothJake(totalDist_avgCoor(:, currInd), 3, 7);
    cumTime = cumsum(elapsedTime);
    timeInd1 = find(cumTime <= cumTime(end) - 1, 1, 'last');
    timeInd2 = find(cumTime <= cumTime(end) - 0.2, 1, 'last');
    smoothCurr = smoothCurr(timeInd2:end);
    smoothCurr = mean(smoothCurr);

    cFlag = false;

    for p = 1:length(otherInd)
        diffDist = diff(totalDist_avgCoor(:, otherInd(p)));
        diffDist = smoothJake(diffDist, 3, 7);

        % we don't want to register the decision if the animal has moved
        % significantly towards another platform in the last second or so. 
        
        diffDist = diffDist(timeInd1:end);
        sumDiffDist = sum(diffDist);
        
        smoothOther = smoothJake(totalDist_avgCoor(:, otherInd(p)), 3, 7);
        smoothOther = mean(smoothOther(timeInd2:end));

        if sumDiffDist < -50 || (sumDiffDist < -25 && smoothOther - smoothCurr < 50) % if he is, go to next loop
            elapsedTime(counter, 1) = toc;
            counter = counter + 1;
            flush(u)
            cFlag = true;            
        end
    end
    if cFlag 
        continue
    end

   elapsedTime(counter, 1) = toc;
   platform = platformPosition(counter);
   clear u
   return
end









