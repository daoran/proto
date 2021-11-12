addpath(genpath("proto"));
pkg load symbolic;

% Projection Matrix i
syms Pi0 Pi1 Pi2  Pi3;
syms Pi4 Pi5 Pi6  Pi7;
syms Pi8 Pi9 Pi10 Pi11;

Pi = [Pi0 Pi1 Pi2  Pi3;
      Pi4 Pi5 Pi6  Pi7;
      Pi8 Pi9 Pi10 Pi11];

% Projection Matrix j
syms Pj0 Pj1 Pj2  Pj3;
syms Pj4 Pj5 Pj6  Pj7;
syms Pj8 Pj9 Pj10 Pj11;

Pj = [Pj0 Pj1 Pj2  Pj3;
      Pj4 Pj5 Pj6  Pj7;
      Pj8 Pj9 Pj10 Pj11];

% Image point z_i
syms x_i y_i;

% Image point z_j
syms x_j y_j;

% 3D point p
syms p_x p_y p_z p_w;

Pi1T = Pi(1, :);
Pi2T = Pi(2, :);
Pi3T = Pi(3, :);

Pj1T = Pj(1, :);
Pj2T = Pj(2, :);
Pj3T = Pj(3, :);

A = [y_i * Pi3T - Pi2T;
     x_i * Pi3T - Pi1T;
     y_j * Pj3T - Pj2T;
     x_j * Pj3T - Pj1T];

A' * A
