function path = join_paths(varargin)
  path = varargin{1};

  for i = 2:nargin
    val = varargin{i};

    if path(end) != '/' && val(1) != '/'
      path = strcat(strcat(path, "/"), val);
    else
      path = strcat(path, val);
    end
  end
endfunction
