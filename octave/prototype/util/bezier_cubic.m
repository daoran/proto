function s = bezier_cubic(P0, C0, C1, P1, t)
  % P0 : np.array Start anchor point
  % C0 : np.array First control point
  % C1 : np.array Second control point
  % P1 : np.array End anchor point
  % t : float Parameter
  % s : Position on bezier curve at t
  s = (1 - t)**3 * P0;
  s += 3 * (1 - t)**2 * t * C0;
  s += 3 * (1 - t) * t**2 * C1;
  s += t**3 * P1;
endfunction
