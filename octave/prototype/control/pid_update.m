function output = pid_update(pid, setpoint, actual, dt)
  # Calculate errors
  error = setpoint - actual;
  p.error_sum += error * dt;
  p.error_prev = error;

  # Calculate output
  p.error_p = p.k_p * error;
  p.error_i = p.k_i * p.error_sum;
  p.error_d = p.k_d * (error - p.error_prev) / dt;
  output = p.error_p + p.error_i + p.error_d;

  return output;
endfunction
