classdef pose_t < param_t
  methods
    function pose = pose_t(ts, data)
      assert(all(size(data) == [7, 1]));
      pose@param_t("pose_t", ts, data, 6);
    endfunction
  endmethods
endclassdef
