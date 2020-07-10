addpath(genpath("proto"));
graphics_toolkit("fltk");

% Fiducial pose T_WF
poses = csvread("/tmp/T_WF.csv");
q = [poses(1, 1); poses(1, 2); poses(1, 3); poses(1, 4)];
p = [poses(1, 5); poses(1, 6); poses(1, 7)];
% q = [0.710313515521; -0.00073224806; 0.000344859106; -0.70388497253];
% p = [-0.0234899429762; 0.296941072988; 0.025011621861];
T_WF = tf(q, p);

% Marker poses T_WM
poses = csvread("/tmp/poses.csv");
T_WM = {};
for i = 1:rows(poses)
  q = [poses(i, 1); poses(i, 2); poses(i, 3); poses(i, 4)];
  p = [poses(i, 5); poses(i, 6); poses(i, 7)];
  T_WM{i} = tf(q, p);
end

% Marker to Camera extrinsic
poses = csvread("/tmp/T_MC.csv");
q = [poses(1, 1); poses(1, 2); poses(1, 3); poses(1, 4)];
p = [poses(1, 5); poses(1, 6); poses(1, 7)];
T_MC = tf(q, p);

% First camera pose
T_WC = T_WM{1} * T_MC;

% Plot
figure(1);
hold on;
grid on;
view(3);
draw_frame(T_WF, 0.1);
text(T_WF(1, 4), T_WF(2, 4), T_WF(3, 4) - 0.04, "T_{WF}");

C = euler321(deg2rad([-90.0, 0.0, 0.0]));
r = zeros(3, 1);
T_hack = tf(C, r);

draw_frame(T_WM{1}, 0.1);
text(T_WM{1}(1, 4), T_WM{1}(2, 4), T_WM{1}(3, 4) - 0.04, "T_{WM}");

% draw_frame(T_WC, 0.1);
% text(T_WC(1, 4), T_WC(2, 4), T_WC(3, 4) - 0.04, "T_{WC}");

% for i = 1:rows(poses)
%   if norm(T_WM{i}(1:3, 4)) < 100
%     draw_frame(T_WM{i});
%   end
% end

xlabel("x");
ylabel("y");
zlabel("z");
axis('equal');
ginput();
