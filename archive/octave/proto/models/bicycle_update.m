function bicycle = bicycle_update(bicycle, v, delta, dt)
  % Motion increment in the body frame
  dx_b = [v; 0; v * tan(delta) / bicycle.l] * dt;
  R_IB = rotz(bicycle.state(3));
  bicycle.state = bicycle.state + R_IB * dx_b;
endfunction
