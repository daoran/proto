classdef sb_t < param_t
  properties
  endproperties

  methods
    function sb = sb_t(ts, v, bg, ba)
      assert(all(size(v) == [3, 1]));
      assert(all(size(bg) == [3, 1]));
      assert(all(size(ba) == [3, 1]));
      sb@param_t("sb_t", ts, [v; bg; ba], 9);
    endfunction
  endmethods
endclassdef
