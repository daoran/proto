function imu_buf = imu_buf_init(t, acc, gyr)
  assert(nargin == 0 || nargin == 3);

  if nargin == 0
    imu_buf = {};
    imu_buf.ts = [];
    imu_buf.acc = [];
    imu_buf.gyr = [];
  elseif nargin == 3
    assert(size(t) == [1, 1]);
    assert(size(acc) == [3, 1]);
    assert(size(gyr) == [3, 1]);
    imu_buf = {};
    imu_buf.ts = [t];
    imu_buf.acc = [acc];
    imu_buf.gyr = [gyr];
  endif
endfunction
