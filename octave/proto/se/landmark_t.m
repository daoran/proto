classdef landmark_t < param_t
  methods
    function lm = landmark_t(ts, x)
      assert(all(size(x) == [3, 1]));
      lm@param_t("landmark_t", ts, x, 3);
    endfunction
  endmethods
endclassdef
