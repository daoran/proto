#ifndef PROTO_FS_HPP
#define PROTO_FS_HPP

#include "core.hpp"

namespace proto {

FILE *file_open(const std::string &path,
                const std::string &mode,
                int *nb_rows = nullptr);
void skip_line(FILE *fp);
int file_rows(const std::string &file_path);
int file_copy(const std::string &src, const std::string &dest);
std::string parse_fext(const std::string &path);
std::string parse_fname(const std::string &path);
bool file_exists(const std::string &path);
bool dir_exists(const std::string &path);
int dir_create(const std::string &path);
std::string dir_name(const std::string &path);
std::string strip(const std::string &s, const std::string &target = " ");
std::string strip_end(const std::string &s, const std::string &target = " ");
int create_dir(const std::string &path);
int remove_dir(const std::string &path);
std::string remove_ext(const std::string &path);
int list_dir(const std::string &path, std::vector<std::string> &results);
std::vector<std::string> path_split(const std::string path);
std::string paths_join(const std::string path1, const std::string path2);

} // namespace proto
#endif // PROTO_FS_HPP
