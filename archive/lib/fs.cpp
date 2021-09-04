#include "fs.hpp"

namespace proto {

/**
 * Open file in `path` with `mode` and set `nb_rows`.
 * @returns File pointer on success, nullptr on failure.
 */
FILE *file_open(const std::string &path,
                const std::string &mode,
                int *nb_rows) {
  FILE *fp = fopen(path.c_str(), mode.c_str());
  if (fp == NULL) {
    return nullptr;
  }

  if (nb_rows != nullptr) {
    *nb_rows = file_rows(path);
  }

  return fp;
}

/**
 * Skip file line in file `fp`.
 */
void skip_line(FILE *fp) {
  char header[BUFSIZ];
  char *retval = fgets(header, BUFSIZ, fp);
  if (retval == NULL) {
    FATAL("Failed to skip line!");
  }
}

/**
 * Get number of rows in file.
 * @returns Number of rows in file else -1 for failure.
 */
int file_rows(const std::string &file_path) {
  // Load file
  std::ifstream infile(file_path);
  if (infile.good() != true) {
    return -1;
  }

  // Obtain number of lines
  int nb_rows = 0;
  std::string line;
  while (std::getline(infile, line)) {
    nb_rows++;
  }

  return nb_rows;
}

/**
 * Copy file from path `src` to path `dest.
 *
 * @returns 0 for success else -1 if `src` file could not be opend, or -2 if
 * `dest` file could not be opened.
 */
int file_copy(const std::string &src, const std::string &dest) {
  // Open input path
  FILE *src_file = fopen(src.c_str(), "rb");
  if (src_file == NULL) {
    fclose(src_file);
    return -1;
  }

  // Open output path
  FILE *dest_file = fopen(dest.c_str(), "wb");
  if (dest_file == NULL) {
    fclose(src_file);
    fclose(dest_file);
    return -2;
  }

  // BUFSIZE default is 8192 bytes
  // BUFSIZE of 1 means one chareter at time
  char buf[BUFSIZ];
  while (size_t size = fread(buf, 1, BUFSIZ, src_file)) {
    fwrite(buf, 1, size, dest_file);
  }

  // Clean up
  fclose(src_file);
  fclose(dest_file);

  return 0;
}

/**
 * Return file extension in `path`.
 */
std::string parse_fext(const std::string &path) {
  return path.substr(path.find_last_of("."));
}

/**
 * Return basename
 */
std::string parse_fname(const std::string &path) {
  auto output = path;
  const size_t last_slash_idx = output.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    output.erase(0, last_slash_idx + 1);
  }

  return output;
}

/**
 * Check if file exists
 *
 * @param path Path to file
 * @returns true or false
 */
bool file_exists(const std::string &path) {
  FILE *file;

  file = fopen(path.c_str(), "r");
  if (file != NULL) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

/**
 * Check if path exists
 *
 * @param path Path
 * @returns true or false
 */
bool dir_exists(const std::string &path) {
  DIR *dir = opendir(path.c_str());

  if (dir) {
    closedir(dir);
    return true;
  } else if (ENOENT == errno) {
    return false;
  } else {
    LOG_ERROR("dir_exists() failed! %s", strerror(errno));
    exit(-1);
  }
}

/**
 * Create directory
 *
 * @param path Path
 * @returns 0 for success, -1 for failure
 */
int dir_create(const std::string &path) {
  std::string command = "mkdir -p " + path;
  return system(command.c_str());
}

/**
 * Return directory name
 *
 * @param path Path
 * @returns directory name
 */
std::string dir_name(const std::string &path) {
  std::size_t found = path.find_last_of("/\\");
  return path.substr(0, found);
}

/**
 * Strips a target character from the start and end of a string
 *
 * @param s String to strip
 * @param target Target character to strip
 * @returns Stripped string
 */
std::string strip(const std::string &s, const std::string &target) {
  size_t first = s.find_first_not_of(target);
  if (std::string::npos == first) {
    return s;
  }

  size_t last = s.find_last_not_of(target);
  return s.substr(first, (last - first + 1));
}

/**
 * Strips a target character from the end of a string
 *
 * @param s String to strip
 * @param target Target character to strip
 * @returns Stripped string
 */
std::string strip_end(const std::string &s, const std::string &target) {
  size_t last = s.find_last_not_of(target);
  return s.substr(0, last + 1);
}

/**
 * Create directory
 *
 * @param path Path to directory
 * @returns 0 for success, -1 for failure
 */
int create_dir(const std::string &path) {
  const std::string command = "mkdir -p " + path;
  const int retval = system(command.c_str());
  if (retval == -1) {
    printf("Error creating directory!n");
    return -1;
  }

  return 0;
}

/**
 * Remove directory
 *
 * @param path Path to directory
 * @returns 0 for success, -1 for failure
 */
int remove_dir(const std::string &path) {
  DIR *dir = opendir(path.c_str());
  struct dirent *next_file;

  // pre-check
  if (dir == NULL) {
    return -1;
  }

  // remove files in path
  while ((next_file = readdir(dir)) != NULL) {
    remove(std::string(path + "/" + next_file->d_name).c_str());
  }

  // remove dir
  remove(path.c_str());
  closedir(dir);

  return 0;
}

/**
 * Remove file extension
 *
 * @param path Path to directory
 * @returns File path without extension
 */
std::string remove_ext(const std::string &path) {
  auto output = path;
  const size_t period_idx = output.rfind('.');
  if (std::string::npos != period_idx) {
    output.erase(period_idx);
  }
  return output;
}

/**
 * List directory
 *
 * @param path Path to directory
 * @param results List of files and directories
 * @returns 0 for success, -1 for failure
 */
int list_dir(const std::string &path, std::vector<std::string> &results) {
  struct dirent *entry;
  DIR *dp;

  // Check directory
  dp = opendir(path.c_str());
  if (dp == NULL) {
    return -1;
  }

  // List directory
  while ((entry = readdir(dp))) {
    std::string value(entry->d_name);
    if (value != "." && value != "..") {
      results.push_back(value);
    }
  }

  // Clean up
  closedir(dp);
  return 0;
}

/**
 * Split path into a number of elements
 *
 * @param path Path
 * @returns List of path elements
 */
std::vector<std::string> path_split(const std::string path) {
  std::string s;
  std::vector<std::string> splits;

  s = "";
  for (size_t i = 0; i < path.length(); i++) {
    if (s != "" && path[i] == '/') {
      splits.push_back(s);
      s = "";
    } else if (path[i] != '/') {
      s += path[i];
    }
  }
  splits.push_back(s);

  return splits;
}

/**
 * Combine `path1` and `path2`
 *
 * @param path1 Path 1
 * @param path2 Path 22
 * @returns Combined path
 */
std::string paths_join(const std::string path1, const std::string path2) {
  int dirs_up;
  std::string result;
  std::vector<std::string> splits1;
  std::vector<std::string> splits2;

  // setup
  result = "";
  splits1 = path_split(path1);
  splits2 = path_split(path2);

  // obtain number of directory ups in path 2
  dirs_up = 0;
  for (size_t i = 0; i < splits2.size(); i++) {
    if (splits2[i] == "..") {
      dirs_up++;
    }
  }

  // drop path1 elements as path2 dir ups
  for (int i = 0; i < dirs_up; i++) {
    splits1.pop_back();
  }

  // append path1 to result
  if (path1[0] == '/') {
    result += "/";
  }
  for (size_t i = 0; i < splits1.size(); i++) {
    result += splits1[i];
    result += "/";
  }

  // append path2 to result
  for (size_t i = dirs_up; i < splits2.size(); i++) {
    result += splits2[i];
    result += "/";
  }

  // remove trailing slashes
  for (size_t i = result.length() - 1; i > 0; i--) {
    if (result[i] == '/') {
      result.pop_back();
    } else {
      break;
    }
  }

  return result;
}

} // namespace proto
