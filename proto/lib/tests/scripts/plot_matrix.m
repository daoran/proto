#!/usr/bin/octave -qf
graphics_toolkit("fltk");

% Parser command line args
arg_list = argv();
csv = arg_list{1};

% Parse data
data = csvread(csv, 0, 0);
rows(data)
rank(data)

% Plot figure
figure();
imagesc(data);
colorbar();
% grid on;
ginput()
