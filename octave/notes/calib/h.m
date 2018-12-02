% Measurement function h(): Projects point in fiducial frame to image frame
%
%   T_WS: Sensor pose
%   T_SC0: Sensor to camera extrinsics
%   T_WF: Fiducial pose
%   p_F: Point in fiducial frame
%
function z = h(T_WS, T_SC0, T_WF, p_F)
  p_C0 = (tf_inv(T_SC0) * tf_inv(T_WS) * T_WF * homogeneous(p_F))(1:3);
  z = [p_C0(1) / p_C0(3); p_C0(2) / p_C0(3)];
endfunction
