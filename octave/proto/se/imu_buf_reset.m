function imu_buf = imu_buf_reset(t, acc, gyr)
  assert(nargin == 0 || nargin == 3);

  if nargin == 0
    imu_buf = {};
    imu_buf.ts = [];
    imu_buf.acc = [];
    imu_buf.gyr = [];
  elseif nargin == 3
    imu_buf = {};
    imu_buf.ts = [t];
    imu_buf.acc = [acc];
    imu_buf.gyr = [gyr];
  endif
endfunction
