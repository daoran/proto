#{
  The sequence of the callback can be summarized as follows:

  1. Compute the image pyramid from the image.

  2. Predict the position of the valid MultilevelPatchFeature%s in the current
  image, using the previous 2 image locations of these
  MultilevelPatchFeatures.

  3. Execute 2D patch alignment at the predicted MultilevelPatchFeature
  locations.  If successful the matching status of the MultilevelPatchFeature
  is set to FOUND and its image location is updated.

  4. Prune: Check the MultilevelPatchFeature%s in the MultilevelPatchSet for
  their quality (MultilevelPatchFeature::isGoodFeature()). If a bad quality
  of a MultilevelPatchFeature is recognized, it is set to invalid.

  5. Get new features and add them to the MultilevelPatchSet, if there are too
  little valid MultilevelPatchFeature%s in the MultilevelPatchSet.
  MultilevelPatchFeature%s which are stated invalid are replaced by new
  features.
#}
function tracker = feature_tracker_update(tracker,
                                          cam0_image,
                                          cam1_image,
                                          show_images=false)
  pyramid_levels = 2;

  % cam0_kps = cv.FAST(cam0_image);
  % cam1_kps = cv.FAST(cam1_image);

  % Image pyramid
  cam0_image_pyr = cv.buildPyramid(cam0_image, "MaxLevel", pyramid_levels);
  cam1_image_pyr = cv.buildPyramid(cam1_image, "MaxLevel", pyramid_levels);




  % % Visualize
  % if show_images
  %   % Draw keypoints
  %   cam0_image = cv.drawKeypoints(cam0_image, cam0_kps, "Color", [255, 0, 0]);
  %   cam1_image = cv.drawKeypoints(cam1_image, cam1_kps, "Color", [255, 0, 0]);
  %
  %   % Show
  %   figure(1);
  %   hold on;
  %   subplot(121);
  %   imshow(cam0_image);
  %   subplot(122);
  %   imshow(cam1_image);
  %   ginput();
  % endif
endfunction
