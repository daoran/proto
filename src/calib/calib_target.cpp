#include "prototype/calib/calib_target.hpp"

namespace prototype {

calib_target_t::calib_target_t() {}

calib_target_t::~calib_target_t() {}

std::ostream &operator<<(std::ostream &os, const calib_target_t &target) {
  os << "type: " << target.type << std::endl;
  os << "rows: " << target.tag_rows << std::endl;
  os << "wols: " << target.tag_cols << std::endl;
  os << "size: " << target.tag_size << std::endl;
  os << "spacing: " << target.tag_spacing << std::endl;
  return os;
}

int calib_target_load(calib_target_t &ct, const std::string &target_file) {
  config_t config{target_file};
  if (config.ok == false) {
    LOG_ERROR("Failed to load target file [%s]!", target_file.c_str());
    return -1;
  }
  parse(config, "type", ct.type);
  parse(config, "tag_rows", ct.tag_rows);
  parse(config, "tag_cols", ct.tag_cols);
  parse(config, "tag_size", ct.tag_size);
  parse(config, "tag_spacing", ct.tag_spacing);

  return 0;
}

} //  namespace prototype
