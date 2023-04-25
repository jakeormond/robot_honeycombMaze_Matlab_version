function [duration, robotInput, nEpochs] = getPathTimings(robotInput, timePerGap, timePerLine)
%%
duration = cell(1,2);
for r = 1:2
    if isempty(robotInput{r})
        continue
    end

    duration{r} = NaN(1,length(robotInput{r})-1);
    for e = 1:length(robotInput{r})-1
        turnLinesTemp = robotInput{r}{e}(3:2:end);
        turnLinesTemp(turnLinesTemp > 3) = ...
            6 - turnLinesTemp(turnLinesTemp > 3);

        nGapsTemp = robotInput{r}{e}(2:2:end);

        duration{r}(e) = (timePerGap * sum(nGapsTemp)) + ...
            (timePerLine * sum(turnLinesTemp));
    end
    duration{r}(length(robotInput{r})) = 2;
end

nEpochs = max(cellfun(@length, duration));
for r = 1:2
    while length(robotInput{r}) < nEpochs
        robotInput{r} = [{[]}, robotInput{r}];
        duration{r} = [0, duration{r}];
    end
end