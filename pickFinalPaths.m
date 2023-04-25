function pathsFinal = pickFinalPaths(robot, paths, minLengths, ...
    maxLengths)
%%
statRobot = find(strcmp({robot(:).movState}, 'stationary'));
nonStatRobots = find(~ismember(1:3, statRobot));

minMaxLength = min(maxLengths(:));
minInd = find(maxLengths(:) == minMaxLength);
[p1, i1, d1, i2, d2] = ind2sub(size(maxLengths), minInd);

if length(p1) > 1
    minLength = NaN(length(p1), 1);
    for m = 1:length(p1)
        minLength(m) = minLengths(p1(m), i1(m), d1(m), i2(m), d2(m));
    end
    [~, minInd] = min(minLength);
    p1 = p1(minInd);
    i1 = i1(minInd);
    d1 = d1(minInd);
    i2 = i2(minInd);
    d2 = d2(minInd);
end

p2 = find(~ismember([1 2], p1));

pathsFinal = cell(1,2);
pathsFinal{1} = [robot(nonStatRobots(1)).pos; ...
    paths{1, p1, i1, d1}];
pathsFinal{2} = [robot(nonStatRobots(2)).pos; ...
    paths{2, p2, i2, d2}];