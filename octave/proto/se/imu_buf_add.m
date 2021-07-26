function imu_buf = imu_buf_add(imu_buf, t, acc, gyr)
  assert(isstruct(imu_buf));
  assert(size(t) == [1]);
  assert(size(acc) == [3, 1]);
  assert(size(gyr) == [3, 1]);

  imu_buf.ts = [imu_buf.ts, t];
  imu_buf.acc = [imu_buf.acc, acc];
  imu_buf.gyr = [imu_buf.gyr, gyr];
endfunction
