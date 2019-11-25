function ctraj = ctraj_init(timestamps, positions, orientations)
  assert(length(timestamps) == length(positions));
  assert(length(timestamps) == length(orientations));
  assert(length(timestamps) > 4);

  ctraj = {};
  ctraj.timestamps = timestamps;

  # Create knots
  nb_knots = length(ctraj.timestamps);
  knots = zeros(1, nb_knots);
  ts_start = timestamps(1);
  ts_end = timestamps(end);
  for i = 1:nb_knots
    ts = timestamps(i);
    knots(i) = (ts - ts_start) / (ts_end - ts_start);
  endfor

  % // Prep position data
  % matx_t pos{3, nb_knots};
  % for (size_t i = 0; i < nb_knots; i++) {
  %   pos.block<3, 1>(0, i) = ctraj.positions[i];
  % }
  %
  % // Prep orientation data
  % matx_t rvec{3, nb_knots};
  % Eigen::AngleAxisd aa{ctraj.orientations[0]};
  % rvec.block<3, 1>(0, 0) = aa.angle() * aa.axis();
  %
  % for (size_t i = 1; i < nb_knots; i++) {
  %   const Eigen::AngleAxisd aa{ctraj.orientations[i]};
  %   const vec3_t rvec_k = aa.angle() * aa.axis();
  %   const vec3_t rvec_km1 = rvec.block<3, 1>(0, i - 1);
  %
  %   // Calculate delta from rvec_km1 to rvec_k
  %   vec3_t delta = rvec_k - rvec_km1;
  %   while (delta.squaredNorm() > (M_PI * M_PI)) {
  %     delta -= 2 * M_PI * delta.normalized();
  %   }
  %
  %   // Add new rotation vector
  %   rvec.block<3, 1>(0, i) = rvec_km1 + delta;
  % }
  %
  % // Create splines
  % const int spline_degree = 3;
  % ctraj.pos_spline = SPLINE3D(pos, knots, spline_degree);
  % ctraj.rvec_spline = SPLINE3D(rvec, knots, spline_degree);
endfunction
