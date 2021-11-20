addpath(genpath("proto"));
graphics_toolkit("fltk");

% r_WS = [0.878612; 2.142470; 0.947262];
% q_WS = [0.060514; -0.828459; -0.058956; -0.553641];
% T_WS = tf(q_WS, r_WS);
%
% T_SC0 = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975;
%          0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768;
%          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949;
%          0.0, 0.0, 0.0, 1.0];

% T_WC0 = T_WS * T_SC0;

# Camera world pose
rpy_WC0 = deg2rad([-90, 0, -90]);
C_WC0 = euler321(rpy_WC0);
% r_WC0 = [0;0;0];
% T_WC0 = tf(C_WC0, r_WC0);
%
% % p_W = T_WC0 * p_C0;
% % T_WC1 = T_WC0 * T_C0C1;
%
% figure(1);
% hold on;
% draw_frame(T_WC0);
% % draw_frame(T_WC0);
%
% axis("equal");
% xlabel("x [m]");
% ylabel("y [m]");
% zlabel("z [m]");
% view(3);
% ginput();
%

function q = quatFromEuler(x, y, z)
  ca = cos(x/2);
  cb = cos(y/2);
  cc = cos(z/2);
  sa = sin(x/2);
  sb = sin(y/2);
  sc = sin(z/2);

  q = [ca*cb*cc-sa*cb*sc; ca*sb*sc-sa*sb*cc; ca*sb*cc+sa*sb*sc; sa*cb*cc+ca*cb*sc];
endfunction

q = rot2quat(C_WC0)
q = quatFromEuler(rpy_WC0(1), rpy_WC0(2), rpy_WC0(3))
