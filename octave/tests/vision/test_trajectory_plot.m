addpath(genpath("prototype"));

camera = camera_create([680; 480], 90.0);
chessboard = chessboard_create(nb_rows=4, nb_cols=4, tag_size=0.2);
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
