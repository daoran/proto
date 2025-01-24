#pragma once

#include <stdlib.h>
#include <stdio.h>

#include "data.h"

FILE *gnuplot_init(void); void gnuplot_close(FILE *pipe);
void gnuplot_multiplot(FILE *pipe, const int num_rows, const int num_cols);
void gnuplot_send(FILE *pipe, const char *cmd);
void gnuplot_xrange(FILE *pipe, const real_t xmin, const real_t xmax);
void gnuplot_yrange(FILE *pipe, const real_t ymin, const real_t ymax);
void gnuplot_send_xy(FILE *pipe,
                     const char *data_name,
                     const real_t *xvals,
                     const real_t *yvals,
                     const int n);
void gnuplot_send_matrix(FILE *pipe,
                         const char *data_name,
                         const real_t *A,
                         const int m,
                         const int n);
void gnuplot_matshow(const real_t *A, const int m, const int n);
