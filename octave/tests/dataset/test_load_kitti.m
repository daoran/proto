addpath(genpath("prototype"));

DATA_PATH = "/data/kitti";
data = load_kitti(DATA_PATH, "2011_09_26", "0005");

p_G = data.oxts.p_G;
ts = data.oxts.timestamps;
t = data.oxts.time;

figure()
subplot(211);
plot(p_G(1, :), p_G(2, :), 'r.')
axis("equal");

subplot(212);
plot(t, p_G(3, :), 'r-')
xlim([0, t(end)]);
ginput()
