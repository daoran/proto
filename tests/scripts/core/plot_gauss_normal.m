#!/usr/bin/octave -qf
graphics_toolkit("fltk");

% Parser command line args
arg_list = argv();
csv = arg_list{1};

% Parse data
data = csvread(csv, 0, 0);

% Plot figure
figure();

subplot(311);
hist(data);
title("Gauss Normal");

ginput()
