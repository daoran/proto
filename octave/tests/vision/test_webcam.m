addpath(genpath("prototype"));

% Connect to a camera
camera = cv.VideoCapture();
pause(2);
for i = 1:50
  frame = camera.read;
  % imshow(frame);
end
