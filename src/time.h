#pragma once

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

/** Timestamp Type */
#ifndef timestamp_t
typedef int64_t timestamp_t;
#endif

/** Tic toc macros */
#define TIC(X) struct timespec X = tic()
#define TOC(X) toc(&X)
#define MTOC(X) mtoc(&X)
#define PRINT_TOC(PREFIX, X) printf("[%s]: %.4fs\n", PREFIX, toc(&X))
#define PRINT_MTOC(PREFIX, X) printf("[%s]: %.4fms\n", PREFIX, mtoc(&X))

struct timespec tic(void);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);
timestamp_t time_now(void);

timestamp_t str2ts(const char *ts_str);
double ts2sec(const timestamp_t ts);
timestamp_t sec2ts(const double time_s);
