% getRoboPlatCoor
% select the coordinates of each platform in the robot maze
%%
% cd robot_maze/platformImages

imageFiles = dir("plats*.jpg");

for i = 4:length(imageFiles)
    imageName = imageFiles(i).name;
    plats = regexp(imageName,'\d*','Match');
    plats = str2double(plats{1}):str2double(plats{2});

    imagePlats = imread(imageName);
    htitle = imageName;
    h = figure('Name', imageName);
    imshow(imagePlats)

    p = 1;
    while p <=length(plats)
        roi = drawcircle('StripeColor','y');

        satisf = input('happy with circle? enter for yes, n for no', 's');

        if strcmp(satisf, 'n')
            oldRoi = findobj(h,'Type','images.roi.Circle');
            delete(oldRoi(1));
            continue
        end

        platCoor(plats(p), 1).Centre = round(roi.Center, 1);
        platCoor(plats(p), 1).Radius = round(roi.Radius, 1);
        p = p + 1;
    end

    save platformCoordinates.mat platCoor
    close
end

