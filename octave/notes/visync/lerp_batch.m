function sensor = lerp_batch(interp_buf, sensor)
  lerped_ts = [];
  lerped_data = [];

  % Interpolation start point
  k0 = {};
  k0.ts = sensor.ts(end-1);
  k0.data = sensor.data(:, end-1);

  % Interpolation end point
  k1 = {};
  k1.ts = sensor.ts(end);
  k1.data = sensor.data(:, end);

  % Get last sensor0 timestamp and data
  last_ts = sensor.ts(end);
  last_data = sensor.data(:, end);

  % Remove last sensor0 timestamp and data
  sensor.ts = sensor.ts(1:end-1);
  sensor.data = sensor.data(:, 1:end-1);

  % Interpolate
  for i = 1:length(interp_buf)
    interp_ts = interp_buf(i);

    if (interp_ts - k0.ts) > 0
      t = (interp_ts - k0.ts) / (k1.ts - k0.ts);
      sensor0_lerped = lerp(k0.data, k1.data, t);

      lerped_ts = [lerped_ts, interp_ts];
      lerped_data = [lerped_data, sensor0_lerped];
    endif
  endfor

  % Add interpolated and last
  sensor.ts = [sensor.ts, lerped_ts, last_ts];
  sensor.data = [sensor.data, lerped_data, last_data];
endfunction
