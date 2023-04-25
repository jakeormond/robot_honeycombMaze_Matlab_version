% compressRobotVideo
%%
movieFile = uigetfile;
video = VideoReader(movieFile);
nFrames = video.NumFrames;
        
compVideoName = [movieFile '_compressed'];
compVideo = VideoWriter(compVideoName, 'Motion JPEG AVI');
compVideo.Quality = 50;

open(compVideo)

for f = 1:nFrames
    frame = readFrame(video);
    frame = rgb2gray(frame);
    frame = real(frame);
    writeVideo(compVideo, frame);
end

close(compVideo)

