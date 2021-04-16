#!/usr/bin/octave -qf
graphics_toolkit("fltk");

% Parser command line args
arg_list = argv();
mvn_csv = arg_list{1};

% Parse data
data = csvread(mvn_csv, 0, 0);

% Plot figure
figure();

subplot(311);
hist(data(1, :));
title("MVN in x");

subplot(312);
hist(data(2, :));
title("MVN in y");

subplot(313);
hist(data(3, :));
title("MVN in z");

ginput()
