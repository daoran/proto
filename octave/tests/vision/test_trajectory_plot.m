addpath(genpath("prototype"));

camera = camera_create([680; 480], 90.0);
chessboard = chessboard_create();
data = trajectory_simulate(camera, chessboard);

figure(1);
hold on;
grid on;
trajectory_plot(data);
xlabel("x");
ylabel("y");
zlabel("z");
axis "equal";
view(3);
ginput();
