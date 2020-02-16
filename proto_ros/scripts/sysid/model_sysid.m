pkg load "signal";

mav = csvread("/tmp/mocap.csv");
imu = csvread("/tmp/imu.csv");
ref = csvread("/tmp/ref.csv");
r_dot = csvread("/tmp/r_dot.csv");

% figure();
% hold on;
% plot(mav(:, 1), mav(:, 5), 'r-', 'linewidth', 2.0);
% plot(mav(:, 1), mav(:, 6), 'g-', 'linewidth', 2.0);
% ginput();

% figure();
% hold on;
% plot(ref(:, 1), ref(:, 2), 'r-', 'linewidth', 2.0);
% plot(ref(:, 1), ref(:, 3), 'g-', 'linewidth', 2.0);
% ginput();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Pitch subsystem ID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys_order = 1;
pitch = [mav(:, 1), mav(:, 5)];
ref_pitch = [ref(:, 1), ref(:, 2)];

smpl_rate = (pitch(2, 1) - pitch(1, 1));
data = iddata(detrend(pitch(:, 1)), detrend(ref_pitch(:, 1)), smpl_rate);
% arx(data, sys_order)
% compare(theta_data, pitch_tf);

% ##%%%%%%%%%%%%%%%%%%%%%%%%%%%% Roll subsystem ID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Use IMU or Vicon ?
% useIMU = true;
% sys_order = 1;
%
% % Get MAV pitch angle (theta)
% if useIMU
%     mav_orientation_eul = zeros(length(imu.q),3);
%     for i=1:length(imu.q)
%          mav_orientation_eul(i,:) = quat2euler(imu.q(i,:));
%     end
%     roll_ts = timeseries(mav_orientation_eul(:,3),imu.t);
% else
%     mav_orientation_eul = zeros(length(vicon.q),3);
%     for i=1:length(vicon.q)
%       mav_orientation_eul(i,:) = quat2euler(vicon.q(i,:));
%     end
%     roll_ts = timeseries(mav_orientation_eul(:,3),vicon.t);
% end
%
% % Get MAV reference pitch angle (theta_ref)
% mav_ref_orientation_eul = zeros(length(ref.q),3);
% for i=1:length(ref.q)
%     mav_ref_orientation_eul(i,:) = quat2euler(ref.q(i,:));
% end
% roll_ref_ts = timeseries(mav_ref_orientation_eul(:,3),ref.t);
%
% % Estimate pitch transfer function
% roll_data = detrend(iddata(roll_ts.Data, roll_ref_ts.Data, Ts));
% roll_tf = tfest(roll_data,sys_order);
% compare(roll_data,roll_tf);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% X_dot subsystem %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% X_dot subsystem
%
% % Use IMU or Vicon ?
% useIMU = false;
% sys_order = 1;
%
% % Get MAV pitch angle (theta)
% if useIMU
%     mav_orientation_eul = zeros(length(imu.q),3);
%     for i=1:length(imu.q)
%          mav_orientation_eul(i,:) = quat2euler(imu.q(i,:));
%     end
%     pitch_ts = timeseries(mav_orientation_eul(:,2),imu.t);
% else
%     mav_orientation_eul = zeros(length(vicon.q),3);
%     for i=1:length(vicon.q)
%       mav_orientation_eul(i,:) = quat2euler(vicon.q(i,:));
%     end
%     pitch_ts = timeseries(mav_orientation_eul(:,2),imu.t);
% end
% x_dot_ts = timeseries(vicon.r_dot_N(:,1),vicon.t);
%
% % Estimate xdot tf
% init_sys = idtf(9.81,[1, 0.2]);
% init_sys.Structure.Numerator.Minimum = 9.81;
% init_sys.Structure.Numerator.Maximum = 9.81;
%
% x_dot_data = iddata(x_dot_ts.Data, pitch_ts.Data, Ts);
% x_dot_tf = tfest(x_dot_data,init_sys);
% compare(x_dot_data,x_dot_tf);
%
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Y_dot subsystem %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Y_dot subsystem
%
% % Use IMU or Vicon ?
% useIMU = true;
% sys_order = 1;
%
% % Get MAV pitch angle (theta)
% if useIMU
%     mav_orientation_eul = zeros(length(imu.q),3);
%     for i=1:length(imu.q)
%          mav_orientation_eul(i,:) = quat2euler(imu.q(i,:));
%     end
%     roll_ts = timeseries(mav_orientation_eul(:,3),imu.t);
% else
%     mav_orientation_eul = zeros(length(vicon.q),3);
%     for i=1:length(vicon.q)
%       mav_orientation_eul(i,:) = quat2euler(vicon.q(i,:));
%     end
%     roll_ts = timeseries(mav_orientation_eul(:,3),imu.t);
% end
% y_dot_ts = timeseries(vicon.r_dot_N(:,2),vicon.t);
%
% % Estimate ydot tf
% y_dot_data = iddata(y_dot_ts.Data, roll_ts.Data, Ts);
%
% init_sys = idtf(-9.81,[1, 0.2]);
% init_sys.Structure.Numerator.Minimum = -9.81;
% init_sys.Structure.Numerator.Maximum = -9.81;
%
% y_dot_tf = tfest(y_dot_data,init_sys);
% compare(y_dot_data,y_dot_tf);
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Z_dot subsystem %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% Z_dot subsystem
%
% z_dot_ts = timeseries(vicon.r_dot_B(:,3),vicon.t);
% thrust_ref_ts = timeseries((ref.Tr + vicon.g)*g,vicon.t);
%
% z_dot_data = iddata(z_dot_ts.Data, thrust_ref_ts.Data, Ts);
% init_sys = idtf(1,[1, 0.2]);
% init_sys.Structure.Numerator.Minimum = 0.7;
% init_sys.Structure.Numerator.Maximum = 1.3;
%
% z_dot_tf = tfest(z_dot_data,init_sys);
% compare(z_dot_data,z_dot_tf);
