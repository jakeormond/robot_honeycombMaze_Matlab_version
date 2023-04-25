% plotPlatCoor_Update
%%
load platformCoordinates_Updated.mat
load platformCoordinates.mat


figure

for p = 1:length(platCoor_updated)
    if isempty(platCoor(p).Centre)
        continue
    end
    
    if ~isempty(platCoor_updated(p).Centre)
        x = platCoor_updated(p).Centre(1);
        y = platCoor_updated(p).Centre(2);
        r = platCoor_updated(p).Radius;
        
        circle(x,y,r, 'k');
        hold on
    end
    %%
    x = platCoor(p).Centre(1);
    y = platCoor(p).Centre(2);
    r = platCoor(p).Radius;
    
    % transform the old coordinates after looking at data
    % There are different ways in which the image may have shifted, and
    % these will determine how to transform. 
    
    % If there is an area of good overlap, then we could model it as having
    % stretched from that point. 
    x = x + (y - 900)*0.01;
    y = y - (x-1800)*0.02;
   % 

    circle(x,y,r, 'r');
    hold on
    
    platCoor(p).Centre(1) = x;
    platCoor(p).Centre(2) = y;
end

save platCoorUpdate_July17.mat platCoor
%% 





%%
function h = circle(x,y,r, colour)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit, colour);
hold off
end