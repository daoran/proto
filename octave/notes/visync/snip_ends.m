function [accel, gyro] = snip_ends(accel, gyro)
  % Trim the start
  if accel.ts(1) > gyro.ts(1)
    gyro.ts = gyro.ts(2:end);
    gyro.data = gyro.data(:, 2:end);
  endif
  if accel.ts(1) < gyro.ts(1)
    accel.ts = accel.ts(2:end);
    accel.data = accel.data(:, 2:end);
  endif

  % Trim the end
  if accel.ts(end) < gyro.ts(end)
    gyro.ts = gyro.ts(1:end-1);
    gyro.data = gyro.data(:, 1:end-1);
  endif
  if accel.ts(end) > gyro.ts(end)
    accel.ts = accel.ts(1:end-1);
    accel.data = accel.data(:, 1:end-1);
  endif
endfunction
