function sb = sb_init(ts, v, ba, bg)
  assert(size(v) == [3, 1]);
  assert(size(ba) == [3, 1]);
  assert(size(bg) == [3, 1]);

  sb.fixed = false;
  sb.type = "sb";
  sb.ts = ts;
  sb.param = [v; ba; bg];
  sb.min_dims = 9;
endfunction
