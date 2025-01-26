#include "munit.h"
#include "xyz.h"
#include "xyz_gnuplot.h"

int test_gnuplot_xyplot(void) {
  // Start gnuplot
  FILE *gnuplot = gnuplot_init();

  // First dataset
  {
    int num_points = 5;
    real_t xvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    real_t yvals[5] = {5.0, 3.0, 1.0, 3.0, 5.0};
    gnuplot_send(gnuplot, "set title 'Plot 1'");
    gnuplot_send_xy(gnuplot, "$DATA1", xvals, yvals, num_points);
  }

  // Second dataset
  {
    int num_points = 5;
    real_t xvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    real_t yvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    gnuplot_send_xy(gnuplot, "$DATA2", xvals, yvals, num_points);
  }

  // Plot both datasets in same plot
  gnuplot_send(gnuplot, "plot $DATA1 with lines, $DATA2 with lines");

  // Clean up
  gnuplot_close(gnuplot);

  return 0;
}

int test_gnuplot_multiplot(void) {
  // Start gnuplot
  FILE *gnuplot = gnuplot_init();

  // Setup multiplot
  const int num_rows = 1;
  const int num_cols = 2;
  gnuplot_multiplot(gnuplot, num_rows, num_cols);

  // First plot
  {
    int num_points = 5;
    real_t xvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    real_t yvals[5] = {5.0, 3.0, 1.0, 3.0, 5.0};
    gnuplot_send(gnuplot, "set title 'Plot 1'");
    gnuplot_send_xy(gnuplot, "$DATA1", xvals, yvals, num_points);
    gnuplot_send(gnuplot, "plot $DATA1 title 'data1' with lines lt 1");
  }

  // Second plot
  {
    int num_points = 5;
    real_t xvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    real_t yvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    gnuplot_send(gnuplot, "set title 'Plot 2'");
    gnuplot_send_xy(gnuplot, "$DATA2", xvals, yvals, num_points);
    gnuplot_send(gnuplot, "plot $DATA2 title 'data1' with lines lt 2");
  }

  // Clean up
  gnuplot_close(gnuplot);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_gnuplot_xyplot);
  MU_ADD_TEST(test_gnuplot_multiplot);
}
MU_RUN_TESTS(test_suite)
