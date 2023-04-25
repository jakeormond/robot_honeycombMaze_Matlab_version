% saveJPEGfromVideo
%%
trialVideoName = uigetfile;
video = VideoReader(trialVideoName);
frame = readFrame(video);
% image(frame)
ind2remove = regexp(trialVideoName, '.avi');
imageName = trialVideoName(1:ind2remove - 1);

imwrite(frame, [imageName '.jpeg'])


% set(gcf, 'PaperPositionMode', 'auto');
% % print('-depsc2', [htitle '.eps'])
% print('-djpeg', [imageName '.jpeg']);


%%
currTimes = [75, 95, 106, 125, 181];

for t = 1:length(currTimes)
    video.CurrentTime = currTimes(t);
    vidFrame = readFrame(video);
    
    vidFrame = vidFrame(800:1400, 1200:1900, :);
    
    imageName = ['frame' num2str(t)];
    imwrite(vidFrame, [imageName '.jpeg'])
end
