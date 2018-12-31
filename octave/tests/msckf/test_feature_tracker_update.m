addpath(genpath("prototype"));

% Setup camera data
data_path = "/data/euroc_mav/MH_01_easy";
data = load_euroc(data_path);

% Iterate through images
show_images = false;
for i = 1:10
  cam0_image = cv.imread(data.cam0.image_paths{i});
  cam1_image = cv.imread(data.cam1.image_paths{i});

  if show_images
    figure(1);
    hold on;
    subplot(121);
    imshow(cam0_image);
    subplot(122);
    imshow(cam1_image);
    ginput();
  endif
endfor
