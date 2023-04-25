function nextPlats = pickNextPlatforms_largeGoal(prevChoices, robot, platformMap, ...
    excludedPlats, all_goal_plats, difficulty)

%%
statRobot = find(strcmp({robot(:).movState}, 'stationary'));
counter = 0;
while true
    [nextPlats, possPlats] = getNextPlats(robot(statRobot).pos, ...
        [robot(~ismember(1:3,statRobot)).pos], platformMap, excludedPlats);
    nextPlats = sort(nextPlats);
    counter = counter + 1;
    
    if length(possPlats) > length(nextPlats)        
        if ~isempty(prevChoices)
            if ~isempty(find(prevChoices(:,1) == nextPlats(1) & ...
                    prevChoices(:,2) == nextPlats(2), 1)) && ...
                    strcmp(difficulty, 'hard')
                continue
            end
        end
    end

    % calculate distance of all platforms to the goal. At least one of
    % the offered platforms need to be closer to the goal than the
    % animal's current location if difficulty set to 'hard'; both must
    % be closer if set to 'easy', unless at the goal.
    currDistance = cartesianDistance_nearest(robot(statRobot).pos, ...
        all_goal_plats, platformMap);
    
    dist1 = cartesianDistance_nearest(nextPlats(1), ...
        all_goal_plats, platformMap);
    dist2 = cartesianDistance_nearest(nextPlats(2), ...
        all_goal_plats, platformMap);   

    if (strcmp(difficulty, 'easy') && dist1 < currDistance ...
            && dist2 < currDistance) || (strcmp(difficulty, 'hard') ...
            && (dist1 < currDistance || dist2 < currDistance)) || ...
            (strcmp(difficulty, 'easy') && (dist1 == 0 || dist2 == 0)) || ...
            (counter > 30 && (dist1 < currDistance || dist2 < currDistance))        

        break
    end
end