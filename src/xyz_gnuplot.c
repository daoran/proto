#include "xyz_gnuplot.h"

FILE *gnuplot_init(void) { return popen("gnuplot -persistent", "w"); }

void gnuplot_close(FILE *pipe) { fclose(pipe); }

void gnuplot_multiplot(FILE *pipe, const int num_rows, const int num_cols) {
  fprintf(pipe, "set multiplot layout %d, %d\n", num_rows, num_cols);
}

void gnuplot_send(FILE *pipe, const char *cmd) { fprintf(pipe, "%s\n", cmd); }

void gnuplot_xrange(FILE *pipe, const double xmin, const double xmax) {
  fprintf(pipe, "set xrange [%f:%f]\n", xmin, xmax);
}

void gnuplot_yrange(FILE *pipe, const double ymin, const double ymax) {
  fprintf(pipe, "set yrange [%f:%f]\n", ymin, ymax);
}

void gnuplot_send_xy(FILE *pipe,
                     const char *data_name,
                     const double *xvals,
                     const double *yvals,
                     const int n) {
  fprintf(pipe, "%s << EOD \n", data_name);
  for (int i = 0; i < n; i++) {
    fprintf(pipe, "%lf %lf\n", xvals[i], yvals[i]);
  }
  fprintf(pipe, "EOD\n");
}

void gnuplot_send_matrix(FILE *pipe,
                         const char *data_name,
                         const double *A,
                         const int m,
                         const int n) {
  // Start data
  fprintf(pipe, "%s << EOD \n", data_name);

  // Print first row with column indices
  fprintf(pipe, "%d ", n);
  for (int j = 0; j < n; j++) {
    fprintf(pipe, "%d ", j);
  }
  fprintf(pipe, "\n");

  // Print rows here first number is row index
  for (int i = 0; i < m; i++) {
    fprintf(pipe, "%d ", i);
    for (int j = 0; j < n; j++) {
      fprintf(pipe, "%lf ", A[(i * n) + j]);
    }
    fprintf(pipe, "\n");
  }

  // End data
  fprintf(pipe, "EOD\n");
}

void gnuplot_matshow(const double *A, const int m, const int n) {
  // Open gnuplot
  FILE *gnuplot = gnuplot_init();

  // Set color scheme
  gnuplot_send(gnuplot, "set palette gray");

  // Set x and y tic labels
  gnuplot_send(gnuplot, "set size square");
  gnuplot_send(gnuplot, "set xtics format ''");
  gnuplot_send(gnuplot, "set x2tics");
  gnuplot_send(gnuplot, "set yrange [* : *] reverse");
  gnuplot_send(gnuplot, "set autoscale x2fix");
  gnuplot_send(gnuplot, "set autoscale yfix");
  gnuplot_send(gnuplot, "set autoscale cbfix");

  // Plot
  gnuplot_send_matrix(gnuplot, "$A", A, m, n);
  gnuplot_send(gnuplot, "plot $A matrix with image notitle axes x2y1");
  gnuplot_send(gnuplot, "pause mouse close");

  // Close gnuplot
  gnuplot_close(gnuplot);
}
