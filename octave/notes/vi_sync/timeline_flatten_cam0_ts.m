function [cam0_ts] = timeline_flatten_cam0_ts(timeline)
  cam0_type = 0;
  cam1_type = 1;
  accel0_type = 2;
  gyro0_type = 3;

  cam0_ts = [];

  for i = 1:length(timeline)
    ts = timeline(i).ts;
    event_type = timeline(i).type;
    for j = 1:length(event_type)
      if event_type(j) == cam0_type
        cam0_ts = [cam0_ts, ts];
      endif
    endfor
  endfor
endfunction
