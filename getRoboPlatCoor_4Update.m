% getRoboPlatCoor_4Update
% select the coordinates of each platform in the robot maze
%%
if exist('platformCoordinates_Updated.mat', 'file')
    load platformCoordinates_Updated.mat
end

imageFile = uigetfile('.jpeg');
imagePlats = imread(imageFile);

htitle = imageFile;
h = figure('Name', imageFile);
imshow(imagePlats)

p = input('what platform?\n');

roi = drawcircle('StripeColor','y');

platCoor_updated(p, 1).Centre = round(roi.Center, 1);
platCoor_updated(p, 1).Radius = round(roi.Radius, 1);

save platformCoordinates_Updated.mat platCoor_updated
close


