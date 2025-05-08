#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include <inttypes.h>

// Settings
#ifndef PRECISION
#define PRECISION 2
#endif

// Float Precision
#ifndef REAL_TYPE
#define REAL_TYPE
#if PRECISION == 1
typedef float real_t;
#elif PRECISION == 2
typedef double real_t;
#else
#error "Floating Point Precision not defined!"
#endif
#endif

// Timestamp Type
#ifndef TIMESTAMP_TYPE
#define TIMESTAMP_TYPE
typedef int64_t timestamp_t;
#endif

