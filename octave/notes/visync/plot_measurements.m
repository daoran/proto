function plot_measurements(plot_title, ts, data, yunit)
  hold on;
  plot(ts, data(1, :), "r-x", "linewidth", 2.0);
  plot(ts, data(2, :), "g-x", "linewidth", 2.0);
  plot(ts, data(3, :), "b-x", "linewidth", 2.0);
  title(plot_title)
	xlabel("time [s]")
	ylabel(yunit)
	grid on;
  hold off;
endfunction
