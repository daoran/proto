pkg load symbolic;

syms cpsi;  # cos(psi)
syms spsi;  # sin(psi)

syms ctheta;  # cos(theta)
syms stheta;  # sin(theta)

syms cphi;  # cos(phi)
syms sphi;  # sin(phi)

# Rotation about the z-axis
Rz = [cpsi, -spsi, sym(0);
      spsi, cpsi, sym(0);
      sym(0), sym(0), sym(1)];

# Rotation about the y-axis
Ry = [ctheta, sym(0), stheta;
      sym(0), sym(1), sym(0);
      -stheta, sym(0), ctheta];

# Rotation about the x-axis
Rx = [sym(1), sym(0), sym(0);
      sym(0), cphi, -sphi;
      sym(0), sphi, cphi];

# Tait-Bryan
Rzyx = Rz * Ry * Rx
