% plotPlatCoor_UpdateV2
%%
load platformCoordinates_Updated.mat
load platformCoordinates.mat

x1 = [];
x2 = [];

y1 = [];
y2 = [];

for p = 1:length(platCoor_updated)
    if isempty(platCoor_updated(p).Centre)
        continue
    end

    x = platCoor_updated(p).Centre(1);
    y = platCoor_updated(p).Centre(2);

    x1 = [x1; x];
    y1 = [y1; y];
    
end

%%
platList = [];
for p = 1:length(platCoor_updated)
    if isempty(platCoor_updated(p).Centre)
        continue
    end
    
    x = platCoor(p).Centre(1);
    y = platCoor(p).Centre(2);

    x2 = [x2; x];
    y2 = [y2; y];
end
%% 
x3 = [];
y3 = [];
platList = [];
for p = 1:length(platCoor)
    if isempty(platCoor(p).Centre)
        continue
    end

    x = platCoor(p).Centre(1);
    y = platCoor(p).Centre(2);
    
    if x < min(x2) || x > max(x2) || y < min(y2) || y > max(y2)
        continue
    end

    platList = [platList; p];

    x3 = [x3; x];
    y3 = [y3; y];
end

[X2, Y2] = meshgrid(x2, y2);
[X3, Y3] = meshgrid(x3, y3);

xIntp = interp2(X2,Y2,x1,X3,Y3);
yIntp = interp2(X2,Y2,y1,X3,Y3);

%%
figure
counter = 0;
for p = 1:length(platCoor_updated)
    if isempty(platCoor_updated(p).Centre)
        continue
    end

    x = platCoor_updated(p).Centre(1);
    y = platCoor_updated(p).Centre(2);
    r = platCoor_updated(p).Radius;

    circle(x,y,r, 'k');
    hold on
    %%
    r = platCoor(p).Radius;
    
    counter = counter + 1;
    x = xIntp(counter);
    y = yInterp(counter);

    circle(x,y,r, 'r');
    hold on
end
%%
function h = circle(x,y,r, colour)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit, colour);
hold off
end