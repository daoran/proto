function sb = sb_init(ts, v, bg, ba)
  assert(size(v) == [3, 1]);
  assert(size(bg) == [3, 1]);
  assert(size(ba) == [3, 1]);

  sb.type = "sb";
  sb.ts = ts;
  sb.param = [v; bg; ba];
  sb.min_dims = 9;
endfunction
