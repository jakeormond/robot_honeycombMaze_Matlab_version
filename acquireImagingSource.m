% acquireImagingSource
% 
%%
imaqhwinfo
info = imaqhwinfo('winvideo');
vid = videoinput('winvideo',1,'RGB24_2448x2048');
vid = videoinput('winvideo',2,'RGB24_2448x2048');
preview(vid)
stoppreview(vid) 

img = getsnapshot(vid);