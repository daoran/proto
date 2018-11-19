addpath(genpath("prototype"));

% IMU to cam0 and cam1 transforms
T_SC0 = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975;
				 0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768;
				 -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949;
				 0.0, 0.0, 0.0, 1.0];
T_SC1 = [0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
				 0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
				 -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
				 0.0, 0.0, 0.0, 1.0];

% Stereo camera extrinsics
T_C0C1 = inv(T_SC0) * T_SC1;

% Calculate IMU attitude using gravity vector and accelerometer reading
g = [0; 0; -9.81];  % Gravity vector
a = [9.2681; -0.310816; -3.14984];  % Accelerometer reading
q = vecs2quat(a, -g);
R_WS = quat2rot(q);

% u = normalize(a);
% v = normalize(-g);
% axis_angle = vecs2axisangle(u, v);
% half_norm = 0.5 * norm(-axis_angle);
% dq_xyz = sinc(half_norm / pi) * 0.5 * -axis_angle;
% dq_w = cos(half_norm);
% dq = [dq_w; dq_xyz];
% q = normalize(quatmul(dq, [1; 0; 0; 0]));
% R_WS = quat2rot(q)

% Assuming IMU is at the origin
T_WS = eye(4);
T_WS(1:3, 1:3) = R_WS;
T_WS(1:3, 4) = [0; 0; 0];

% Plot
figure(1);
hold on;
grid on;
view(3);
draw_frame(T_WS(1:3, 1:3), 0.1);
draw_camera(T_WS * T_SC0, scale=0.05, 'r');
draw_camera(T_WS * T_SC1, scale=0.05, 'g');
xlabel("x");
ylabel("y");
zlabel("z");
axis('equal');
ginput();
