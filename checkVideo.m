

contents = dir;
contents = {contents(:).name};
videoInd = find(cellfun(@(x) ~isempty(x), (regexp(contents, '\w*.avi'))));
video = VideoReader(contents{videoInd});
frame = readFrame(video);
figure
imshow(frame)

