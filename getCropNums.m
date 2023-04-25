function cropNums  = getCropNums(platforms, platCoor)

platCentre = nan(length(platforms),2);
for p = 1:length(platforms)
    platCentre(p,:) = platCoor(platforms(p)).Centre;
end

platCentre = round(mean(platCentre, 1));

platX = mean(min(platCentre(:,1)), max(platCentre(:,1)));
platY = mean(min(platCentre(:,2)), max(platCentre(:,2)));

cropWidth = 600;
cropHeight = 600;

cropX = platX - cropWidth/2;
cropY = platY - cropHeight/2;

if cropX + cropWidth > 2448
    diffCropX = cropX + cropWidth - 2448;
    cropX = cropX - diffCropX;
end

if cropY + cropHeight > 2048
    diffCropY = cropY + cropHeight - 2048;
    cropY = cropY - diffCropY;
end

cropNums = [cropX; cropY; cropWidth; cropHeight];


