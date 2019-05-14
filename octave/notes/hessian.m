pkg load symbolic;

syms Ixx Iyy Ixy;
H = [Ixx, Ixy; Ixy', Iyy];
eig(H)
