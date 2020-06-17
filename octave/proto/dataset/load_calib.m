function data = load_calib(data_path)
  cam0_data = strcat(data_path, "/cam0/data");
  cam0_csv = strcat(data_path, "/cam0/data.csv");
  marker0_csv = strcat(data_path, "/marker0/data.csv");

  % Load camera data
  [cam0_ts, cam0_images] = textread(cam0_csv, ...
                                    "%f %s", ...
                                    "delimiter", ",", ...
                                    "headerlines", 1);
  nb_cam0_images = rows(cam0_images);
  for i = 1:nb_cam0_images
    cam0_images(i, 1) = strcat(cam0_data, "/", cam0_images{i});
  end

  % Load vicon data
  [marker0_ts, ...
   p_RS_S_x, p_RS_S_y, p_RS_S_z, ...
   q_RS_w, q_RS_x, q_RS_y, q_RS_z] = textread( ...
    marker0_csv, ...
    "%f %f %f %f %f %f %f %f", ...
    "delimiter", ",", ...
    "headerlines", 1 ...
  );
  p_RS_S = [p_RS_S_x'; p_RS_S_y'; p_RS_S_z'];
  q_RS_S = [q_RS_w'; q_RS_x'; q_RS_y'; q_RS_z'];
  nb_marker0_measurements = rows(marker0_ts);

  % Form struct
  cam0_ts_min = min(cam0_ts);
  marker0_ts_min = min(marker0_ts);
  t0 = min(cam0_ts_min, marker0_ts_min);

  data.cam0.data_path = cam0_data;
  data.cam0.image_paths = cam0_images;
  data.cam0.ts = cam0_ts;
  cam0_ts_start = data.cam0.ts(1) * ones(nb_cam0_images);
  data.cam0.time = (data.cam0.ts - t0) * 1.0e-9;

  data.marker0.ts = marker0_ts;
  marker0_ts_start = data.marker0.ts(1) * ones(nb_marker0_measurements, 1);
  data.marker0.time = (data.marker0.ts - t0) * 1.0e-9;
  data.marker0.p_RS_S = mat2cell(p_RS_S, 3, ones(1, columns(p_RS_S)));
  data.marker0.q_RS_S = mat2cell(q_RS_S, 4, ones(1, columns(q_RS_S)));
  data.marker0.nb_measurements = length(data.marker0.time);
endfunction
