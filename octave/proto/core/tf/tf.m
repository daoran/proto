function T = tf(varargin)
  rot = eye(3);
  trans = zeros(3, 1);

  % Parse arguments
  assert(length(varargin) == 1 || length(varargin) == 2);

  if length(varargin) == 1
    % Input is a 4x4 transformation matrix
    pose = varargin{1};
    assert(all(size(pose) == [7, 1]));
    rot = quat2rot([pose(7); pose(4); pose(5); pose(6)]);
    trans = pose(1:3);

  elseif length(varargin) == 2
    % Input is a rotation matrix + translation vector
    rot = varargin{1};
    trans = varargin{2};
    assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
    assert(size(trans) == [3, 1]);
    if size(rot) == [4, 1]
      rot = quat2rot(rot);
    endif
  endif

  T = eye(4, 4);
  T(1:3, 1:3) = rot;
  T(1:3, 4) = trans;
endfunction
