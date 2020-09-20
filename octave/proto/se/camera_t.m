classdef camera_t < param_t
  properties
    cam_idx = 0;
    resolution = [0; 0];
    proj_model = "";
    dist_model = "";
  endproperties

  methods
    function camera = camera_t(cam_idx, resolution,
                               proj_model, dist_model,
                               proj_params, dist_params)
      camera@param_t("camera_t", 0, [proj_params; dist_params], 8);
      camera.cam_idx = cam_idx;
      camera.resolution = resolution;
    endfunction
  endmethods
endclassdef
