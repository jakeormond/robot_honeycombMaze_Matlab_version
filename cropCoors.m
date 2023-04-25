% cropCoors
%%
if ~exist('cropNums', 'var')
    cropNums = [50; 50; 500; 300];
end

if cropNums(1) <= 50
    cropNums(1) = 500;
    cropNums(2) = 500;
else
    cropNums(1) = 50;
    cropNums(2) = 50;
end

writematrix(cropNums, 'cropNums.csv')