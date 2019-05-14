function hp = homogeneous(p)
  hp = [p(1,:); p(2,:); p(3,:); ones(1, columns(p))];
endfunction
