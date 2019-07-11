function accel = lerp_batch(interp_buf, accel)
  lerped_ts = [];
  lerped_data = [];

  % Interpolation start point
  k0 = {};
  k0.ts = accel.ts(end-1);
  k0.data = accel.data(:, end-1);

  % Interpolation end point
  k1 = {};
  k1.ts = accel.ts(end);
  k1.data = accel.data(:, end);

  % Remove last accel0 timestamp and data
  accel.ts = accel.ts(1:end-1);
  accel.data = accel.data(:, 1:end-1);

  % Interpolate
  for i = 1:length(interp_buf)
    interp_ts = interp_buf(i);

    % Lerp
    if (interp_ts - k0.ts) > 0
      t = (interp_ts - k0.ts) / (k1.ts - k0.ts);
      accel0_lerped = lerp(k0.data, k1.data, t);

      % Add interpolated point to buffer
      lerped_ts = [lerped_ts, interp_ts];
      lerped_data = [lerped_data, accel0_lerped];
    endif
  endfor

  accel.ts = [accel.ts, lerped_ts];
  accel.data = [accel.data, lerped_data];
endfunction

