% getPlatCoors_auto
%
%%
if exist('platformCoordinates_Auto.mat', 'file')
    load platformCoordinates_Auto.mat
else
    platformCoordinates_Auto = [];
end

filenames = dir('platforms_*.mat');

for i = 1:length(filenames)
    fname = filenames(i).name;

    numEndInd =  regexp(fname, '_');
    platInd = numEndInd + 1;
    extInd = regexp(fname, '.mat');
    numEndInd = [numEndInd(2:end), extInd] - 1;

    plats = NaN(5,1);
    for p = 1:5
        plats(p) = str2double(fname(platInd(p):numEndInd(p)));
    end

    load(filenames(i).name);
    img = rgb2gray(img);
    imshow(img)
    [imgCropped, cropRect] = imcrop(img);
    close

    thresh1 = 180; 
    thresh2 = 70;
    counter = 0;
    while true
        
        imgAdjusted = imgCropped;
        imgAdjusted(imgAdjusted > thresh1) = 0;
        imgAdjusted(imgAdjusted < thresh2) = 0;
        imgAdjusted(imgAdjusted > thresh2) = 255;
        
        imshow(imgAdjusted)
        [centers,radii] = imfindcircles(imgAdjusted, [65 80], ...
            'ObjectPolarity','bright', 'Sensitivity', 0.99);
        
        [~, sortInd] = sort(centers(:, 2));
        centers = centers(sortInd, :);
        radii = radii(sortInd);
        
        h = viscircles(centers,radii);
        
        if length(centers) <= 6 || counter > 0
            break
            
        else
            thresh2 = 45;
            counter = counter + 1;
        end        
    end
    
    pause(1)
    satisf = input('happy with detected platforms? y/n', 's');

    if satisf == 'n'
        while true

            falseFlag = input('any false detections? y/n', 's');

            if falseFlag == 'y'
                falseDetect = input('enter false detections as list ([num1, ..., numN])');
                centers(falseDetect, :) = [];
                radii(falseDetect) = [];

                close all

                imshow(imgAdjusted)
                h = viscircles(centers,radii);
                pause(1)
            end

            missingFlag = input('were any platforms missed? y/n', 's');

            if missingFlag == 'y'
                while true
                    missingPlat = input('enter missing platforms one at a time in order (1-5)');

                    if isempty(missingPlat)
                        break
                    end

                    disp('draw circle')

                    while true
                        roi = drawcircle('StripeColor','y');
                        satisf = input('happy with circle? enter for yes, n for no', 's');
                        if strcmp(satisf, 'n')
                            oldRoi = findobj(h,'Type','images.roi.Circle');
                            delete(oldRoi(1));
                            continue
                        else
                            break
                        end
                    end

                    if missingPlat == 1
                        centers = [roi.Center; centers];
                        radii = [roi.Radius; radii];

                    elseif missingPlat == length(radii) + 1
                        centers = [centers; roi.Center];
                        radii = [radii; roi.Radius];

                    else
                        centers = [centers(1:missingPlat-1, :); roi.Center; centers(missingPlat:end, :)];
                        radii = [radii(1:missingPlat-1); roi.Radius; radii(missingPlat:end)];
                    end

                    close all

                    imshow(imgAdjusted)
                    h = viscircles(centers,radii);
                    pause(1)
                end
            end
            
            satisf = input('happy with detected platforms? y/n', 's');

            if satisf == 'y'
                break
            end
        end
    end

    centers(:,1) = centers(:,1) + cropRect(1);
    centers(:,2) = centers(:,2) + cropRect(2);

    for p = 1:5
        platformCoordinates_Auto(plats(p)).Centre = round(centers(p,:));
        platformCoordinates_Auto(plats(p)).Radius = round(radii(p,:));
    end
    close all
end
%%
imshow(img)
for p = 1:length(platformCoordinates_Auto)
    if isempty(platformCoordinates_Auto(p).Centre)
        continue
    end
    center = platformCoordinates_Auto(p).Centre;
    radius = platformCoordinates_Auto(p).Radius;
    viscircles(center,radius)
    hold on
end
%%
save platformCoordinates_Auto.mat platformCoordinates_Auto

