#pragma once
#include "../Core.hpp"

namespace xyz {

/** AprilGridConfig **/
struct AprilGridConfig {
  int target_id = 0;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;
  int tag_id_offset = 0;

  void getGridIndex(const int tag_id, int &i, int &j) const;
  Vec3 getObjectPoint(const int tag_id, const int corner_index) const;
  std::map<int, Vec3> getObjectPoints() const;
};

} // namespace xyz
