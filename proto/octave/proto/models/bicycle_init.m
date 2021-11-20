function model = bicycle_init(x, y, theta, l)
  model = {};
  model.state = [x; y; theta];
  model.l = l;
endfunction
