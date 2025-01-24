#pragma once

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "macros.h"

size_t string_copy(char *dst, const char *src);
void string_subcopy(char *dst, const char *src, const int s, const int n);
void string_cat(char *dst, const char *src);
char *string_malloc(const char *s);
char *string_strip(char *s);
char *string_strip_char(char *s, const char c);
char **string_split(char *s, const char d, size_t *n);

int **load_iarrays(const char *csv_path, int *num_arrays);
double **load_darrays(const char *csv_path, int *num_arrays);

int *int_malloc(const int val);
float *float_malloc(const float val);
double *double_malloc(const double val);
double *vector_malloc(const double *vec, const double N);

int dsv_rows(const char *fp);
int dsv_cols(const char *fp, const char delim);
char **dsv_fields(const char *fp, const char delim, int *num_fields);
double **
dsv_data(const char *fp, const char delim, int *num_rows, int *num_cols);
void dsv_free(double **data, const int num_rows);

double **csv_data(const char *fp, int *num_rows, int *num_cols);
void csv_free(double **data, const int num_rows);
