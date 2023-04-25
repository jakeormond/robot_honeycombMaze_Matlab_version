function currAndFutureFigure(robot, pathsFinal, platformMap)
%%
statRobot = find(strcmp({robot(:).movState}, 'stationary'));
nonStatRobots = find(~ismember(1:3, statRobot));

htitle = 'platform locations';
h=figure('Name', htitle,'NumberTitle','off','Units',...
    'pixels', 'Position', [600 300 1000 500], 'Color', [1 1 1], 'Visible', 'on');

theta = 0:60:360;
x = cosd(theta);
y = sind(theta);

for i = 1:2
    inRow = NaN(3,1);
    inCol = NaN(3,1);
    plat = NaN(3,1);
    % ipAddr = NaN(3,1);
    for r = 1:3
        if i == 1 || r == statRobot
            currPlat = robot(r).pos;
        else
            movInd = nonStatRobots == r;
            currPlat = pathsFinal{movInd}(end);
        end
        inInd = find(platformMap == currPlat);
        [inRow(r), inCol(r)] = ind2sub(size(platformMap), inInd);
        plat(r) = currPlat;
        % ipAddr(r) = robot(r).ip(end);
    end

    subplot(1,2,i)

    for r = 1:3
        if r == statRobot
            pColour = 'k';
        else
            pColour = 'b';
        end
        relRow = inRow(r) - min(inRow);
        relCol = inCol(r) - min(inCol);

        plot(x + relCol*1.7321, y + relRow, [pColour '-'], 'LineWidth', 3);
        hold on

        % txt1 = num2str(ipAddr(r));
        txt1 = num2str(r);
        text(relCol*1.7321 - .3, relRow - .4, txt1, 'FontSize', 20)
        txt2 = num2str(plat(r));
        text(relCol*1.7321 - .2, relRow + .1, txt2, 'FontSize', 24)
    end
    xlim([-1 5])
    ylim([-1 5])
    box off
    axis off
    set(gca, 'YDir','reverse')

    if i == 1
        title('initial positions')
    else
        title('new positions')
    end
    axis equal
end