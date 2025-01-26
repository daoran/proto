#pragma once

#include <stdio.h>
#include <stdlib.h>

FILE *gnuplot_init(void);
void gnuplot_close(FILE *pipe);
void gnuplot_multiplot(FILE *pipe, const int num_rows, const int num_cols);
void gnuplot_send(FILE *pipe, const char *cmd);
void gnuplot_xrange(FILE *pipe, const double xmin, const double xmax);
void gnuplot_yrange(FILE *pipe, const double ymin, const double ymax);
void gnuplot_send_xy(FILE *pipe,
                     const char *data_name,
                     const double *xvals,
                     const double *yvals,
                     const int n);
void gnuplot_send_matrix(FILE *pipe,
                         const char *data_name,
                         const double *A,
                         const int m,
                         const int n);
void gnuplot_matshow(const double *A, const int m, const int n);
