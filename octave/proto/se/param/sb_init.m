function sb = sb_init(ts, v, bg, ba)
  assert(all(size(v) == [3, 1]));
  assert(all(size(bg) == [3, 1]));
  assert(all(size(ba) == [3, 1]));
  sb.type = "sb";
  sb.ts = ts;
  sb.param = [v; bg; ba];
  sb.min_dims = 9;
endfunction
