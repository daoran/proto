function listing = list_files(target_dir)
  listing = dir(target_dir);

  inds = [];
  n    = 0;
  k    = 1;
  while n < 2 && k <= length(listing)
    if any(strcmp(listing(k).name, {'.', '..'}))
      inds(end + 1) = k;
      n = n + 1;
    end
    k = k + 1;
  end

  listing(inds) = [];
endfunction
