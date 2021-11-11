function compare_matrices(A_title, A, B_title, B, normalize_matrices=0)
  assert(size(A) == size(B));

  if normalize_matrices
    A = normalize(A);
    B = normalize(B);
  endif

  clims = [min(min(min(A)), min(min(B))),
           max(max(max(A)), max(max(B)))];

  figure();

  subplot(311)
  imagesc(A, clims);
  title(A_title);
  colorbar;
  set(gca, "xaxislocation", "top");

  subplot(312)
  imagesc(B, clims);
  title(B_title);
  colorbar;
  set(gca, "xaxislocation", "top");

  subplot(313)
  imagesc(abs(B - A))
  title("Absolute Difference");
  colorbar;
  set(gca, "xaxislocation", "top");
endfunction
