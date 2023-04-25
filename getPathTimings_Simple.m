function [duration, robotInput] = getPathTimings_Simple(robotInput, timePerGap, timePerLine)
%%
turnLinesTemp = robotInput(3:2:end);
turnLinesTemp(turnLinesTemp > 3) = ...
    6 - turnLinesTemp(turnLinesTemp > 3);

nGapsTemp = robotInput(2:2:end);

duration = (timePerGap * sum(nGapsTemp)) + ...
    (timePerLine * sum(turnLinesTemp));


