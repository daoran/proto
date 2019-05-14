addpath(genpath("prototype"));

C_WC = [-1, 0, 0;
        0, 1, 0;
        0, 0, -1;];
r_WC = [0; 0; 0];

T_WC = tf(C_WC, r_WC);

% Plot
figure(1);
hold on;
grid on;
view(3);
draw_frame(T_WC, 0.1);
xlabel("x");
ylabel("y");
zlabel("z");
axis('equal');
ginput();
