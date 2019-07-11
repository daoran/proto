function [accel0_data] = timeline_flatten_accel0_data(timeline)
  cam0_type = 0;
  cam1_type = 1;
  accel0_type = 2;
  gyro0_type = 3;

  accel0_data = [];

  for i = 1:length(timeline)
    event_type = timeline(i).type;
    for j = 1:length(event_type)
      if event_type(j) == accel0_type
        accel0_data = [accel0_data, timeline(i).accel0];
      endif
    endfor
  endfor
endfunction
