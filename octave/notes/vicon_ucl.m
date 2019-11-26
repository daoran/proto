graphics_toolkit("fltk");
addpath(genpath("proto"));

% Load vicon data
printf("Reading CSV data\n");
body_csv = "/data/intel_d435i/calib_vicon_data/body0/data.csv";
[ts, qw, qx, qy, qz px, py, pz] = textread(
  body_csv,
  "%f %f %f %f %f %f %f %f",
  "delimiter", ",",
  "headerlines", 1
);
q_WB = [qw'; qx'; qy'; qz'];
r_WB = [px'; py'; pz'];

printf("Converting timestamp data\n");
ts_start = ts(1);
timestamps = zeros(length(q_WB), 1);
for i = 2:length(q_WB)
  timestamps(i) = (ts(i) - ts_start) * 1e-9;
end

% % Timestamp properties
% ts_start = timestamps(1)
% ts_end = timestamps(end)
% nb_timestamps = length(timestamps)
% ts_hz = nb_timestamps / ts_end
% % -- Timestamp min max difference
% ts_diff = diff(timestamps);
% ts_diff_min = min(ts_diff)
% ts_diff_max = max(ts_diff)
% % -- Outliers
% nb_outliers = 0;
% for i = 1:(nb_timestamps-1)
%   if (ts_diff(i) > 0.012)
%     nb_outliers += 1;
%   end
% end
% nb_outliers
% % -- Plot timestamps min max
% pkg load statistics;
% figure();
% boxplot(diff(timestamps))
% ginput();

% T_BC = [0.000000, 0.000000, 1.000000, 0.000000
%         -1.000000, 0.000000, 0.000000, 0.000000
%         -0.000000, -1.000000, 0.000000, 0.000000
%         0.000000, 0.000000, 0.000000, 1.000000]
%
figure(1);
hold on;
grid on;
view(3);
for i = 1:100:length(q_WB)
  T_WB = tf(q_WB(:, i), r_WB(:, i));
  draw_frame(T_WB);

  % T_WC = T_WB * T_BC;
  % draw_frame(T_WC);
end
xlabel("x");
ylabel("y");
zlabel("z");
axis('equal');
ginput();

% printf("Plotting timestamp data\n");
% figure(2);
% plot(timestamps, px, "r-", "linewidth", 2.0);
% title("Displacement in x");
% ylabel("displacement[m]");
% xlabel("ts");
%
% figure(3);
% plot(timestamps, py, "g-", "linewidth", 2.0);
% title("Displacement in y");
% ylabel("displacement[m]");
% xlabel("ts");
%
% figure(4);
% plot(timestamps, pz, "b-", "linewidth", 2.0);
% title("Displacement in z");
% ylabel("displacement[m]");
% xlabel("ts");

% figure(5);
% plot(timestamps, qx, "r-", "linewidth", 2.0);
% title("qx");
% xlabel("ts");
%
% figure(6);
% plot(timestamps, qy, "g-", "linewidth", 2.0);
% title("qy");
% xlabel("ts");
%
% figure(7);
% plot(timestamps, qz, "b-", "linewidth", 2.0);
% title("qz");
% xlabel("ts");
%
% figure(8);
% plot(timestamps, qw, "b-", "linewidth", 2.0);
% title("qw");
% xlabel("ts");
%
% ginput();
