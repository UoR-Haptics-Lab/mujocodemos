% assume frames is 4dim, and can create an image with 
% image(squueze(frames(1,:,:,:))
% will load frames from moviedata.mat if the file exists
%https://uk.mathworks.com/help/matlab/import_export/convert-between-image-sequences-and-video.html

if exist('moviedata.mat','file'); load moviedata.mat;end
moviesize=size(frames);

%Create New Video with Image Sequence

outputVideo = VideoWriter(fullfile("shuttle_out.avi"));
outputVideo.FrameRate = 30;
open(outputVideo)


for ii = 1:moviesize(1)
   img = squeeze(frames(ii,:,:,:));
   writeVideo(outputVideo,img)
end
% for i = 1:length(imageNames)
%    img = imread(fullfile(workingDir,"images",imageNames{i}));
%    writeVideo(outputVideo,img)
% end

close(outputVideo)

% 
% 
% % Setup
% workingDir = tempname;
% mkdir(workingDir)
% mkdir(workingDir,"images")
% 
% % Create VideoReader Object
% fooVideo = VideoReader("foo.avi");
% % Create Image Sequence
% 
% ii = 1;
% 
% while hasFrame(shuttleVideo)
%    img = readFrame(shuttleVideo);
%    filename = sprintf("%03d",i)+".jpg";
%    fullname = fullfile(workingDir,"images",filename);
%    imwrite(img,fullname)    % Write to a JPEG file (001.jpg, 002.jpg, ..., 121.jpg)
%    i = i+1;
% end