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
function frontend = frontend_update(frontend,
                                    cam0_image,
                                    cam1_image,
                                    show_images=false)
  n_levels = 2;  % Number of image pyramid levels
  patch_size = 8;  % Edge length of patches in pixels. Must be a multiple of 2
	n_max = 100;  % Max number of MultilevelPatchFeatures in a MultilevelPatchSet

	% Detect keypoints
  cam0_kps = cv.FAST(cam0_image);
  cam1_kps = cv.FAST(cam1_image);

  % Image pyramid
  cam0_image_pyr = cv.buildPyramid(cam0_image, "MaxLevel", frontend.n_levels);
  cam1_image_pyr = cv.buildPyramid(cam1_image, "MaxLevel", frontend.n_levels);

	% // Prediction
	% cv::Point2f dc;
	% for(unsigned int i=0;i<nMax_;i++){
	% 	if(fsm_.isValid_[i]){
	% 		dc = 0.75*(fsm_.features_[i].mpCoordinates_->get_c() - fsm_.features_[i].log_previous_.get_c());
	% 		fsm_.features_[i].log_previous_ = *(fsm_.features_[i].mpCoordinates_);
	% 		fsm_.features_[i].mpCoordinates_->set_c(fsm_.features_[i].mpCoordinates_->get_c() + dc);
	% 		if(!fsm_.features_[i].mpMultilevelPatch_->isMultilevelPatchInFrame(pyr_,*(fsm_.features_[i].mpCoordinates_),nLevels_-1,false)){
	% 			fsm_.features_[i].mpCoordinates_->set_c(fsm_.features_[i].log_previous_.get_c());
	% 		}
	% 		fsm_.features_[i].mpStatistics_->increaseStatistics(current_time);
	% 		for(int j=0;j<nCam_;j++){
	% 			fsm_.features_[i].mpStatistics_->status_[j] = UNKNOWN;
	% 		}
	% 	}
	% }

	% Prediction
	% for i = 1:n_max
	% 	if frontend.is_valid(i)
	% 		dc = 0.75 * (fsm_.features_[i].mpCoordinates_->get_c() - fsm_.features_[i].log_previous_.get_c());
  %
	% 	endif
	% endfor
	% (unsigned int i=0;i<nMax_;i++){
	% 	if(fsm_.isValid_[i]){

	% // Get new features, if there are few valid MultilevelPatchFeatures in the MultilevelPatchSet.
	% if(fsm_.getValidCount() < min_feature_count_){
	% 	FeatureCoordinatesVec candidates;
	% 	for(int l = l1; l <= l2; l++){
	% 		pyr_.detectFastCorners(candidates,l,detectionThreshold);
	% 	}
  %
	% 	pruneCandidates(fsm_,candidates,0);
	% 	std::unordered_set<unsigned int> newSet = fsm_.addBestCandidates(
	% 		candidates,pyr_,
	% 		0,
	% 		current_time,
	% 		l1,
	% 		l2,
	% 		max_feature_count_,
	% 		nDetectionBuckets_,
	% 		scoreDetectionExponent_,
	% 		penaltyDistance_,
	% 		zeroDistancePenalty_,
	% 		true,
	% 		0.0
	% 	);
  %
	% 	for(auto it = newSet.begin();it != newSet.end();++it){
	% 		fsm_.features_[*it].log_previous_ = *(fsm_.features_[*it].mpCoordinates_);
	% 		for(int j=0;j<nCam_;j++){
	% 			fsm_.features_[*it].mpStatistics_->status_[j] = TRACKED;
	% 		}
	% 	}
	% }

  % Visualize
  if show_images
    % Draw keypoints
    cam0_image = cv.drawKeypoints(cam0_image, cam0_kps, "Color", [255, 0, 0]);
    cam1_image = cv.drawKeypoints(cam1_image, cam1_kps, "Color", [255, 0, 0]);

    % Show
    figure(1);
    hold on;
    subplot(121);
    imshow(cam0_image);
    subplot(122);
    imshow(cam1_image);
    ginput();
  endif
endfunction
