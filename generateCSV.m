% generateCSV
% this code generates crop parameters for Bonsai. 
% For testing purposes only. 
%%
x = 200;
y = 200;
width = 1000;
height = 600;

if ~exist('cropNums', 'var')
    cropNums = [x; y; width; height];
end

if ~exist('cropNums.csv', 'file')
    if cropNums(1) <= x
        cropNums(1) = cropNums(1) + 100;
    else
        cropNums(1) = cropNums(1) - 100;
    end
    cropNums(2) = cropNums(1);

    cropNums(3) = width;
    cropNums(4) = height;

%     if cropNums(3) <= 200
%         cropNums(3) = cropNums(3) + 200;
%     else
%         cropNums(3) = cropNums(3) - 200;
%     end
% 
%     if cropNums(4) <= 100
%         cropNums(4) = cropNums(4) + 100;
%     else
%         cropNums(4) = cropNums(4) - 100;
%     end

    writematrix(cropNums,'cropNums.csv')
end
