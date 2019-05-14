function q = vecs2quat(u, v)
	cos_theta = transpose(normalize(u)) * normalize(v);
	w = normalize(cross(u, v));

	half_cos = sqrt(0.5 * (1.0 + cos_theta));
	half_sin = sqrt(0.5 * (1.0 - cos_theta));

  qw = half_cos;
  qx = half_sin * w(1);
	qy = half_sin * w(2);
	qz = half_sin * w(3);
	q = [qw; qx; qy; qz];
endfunction
