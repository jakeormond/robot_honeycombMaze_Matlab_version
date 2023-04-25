function nextPlats = getNewPlats(robot, statRobot)
% choose next platforms
%%
nonStatRob = ~ismember(1:3, statRobot);
nonStatPlats = [robot(nonStatRob).pos];

while true
    nextPlats = randsample(innerRing, 2);
    
    if ~isempty(find(~ismember(nextPlats, nonStatPlats), 1))
        break
    end
end
