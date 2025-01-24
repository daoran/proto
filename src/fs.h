#pragma once

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>

#include "macros.h"
#include "logging.h"
#include "data.h"

void path_file_name(const char *path, char *fname);
void path_file_ext(const char *path, char *fext);
void path_dir_name(const char *path, char *dir_name);
char *path_join(const char *x, const char *y);
char **list_files(const char *path, int *num_files);
void list_files_free(char **data, const int n);
char *file_read(const char *fp);
void skip_line(FILE *fp);
status_t file_exists(const char *fp);
status_t file_rows(const char *fp);
status_t file_copy(const char *src, const char *dest);
