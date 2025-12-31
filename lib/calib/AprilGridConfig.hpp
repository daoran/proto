#pragma once
#include "../core/Core.hpp"

namespace cartesian {

/** AprilGridConfig **/
struct AprilGridConfig {
  int target_id = 0;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;
  int tag_id_offset = 0;

  /** Get width */
  double getWidth() const;

  /** Get height */
  double getHeight() const;

  /** Get width and height */
  Vec2 getWidthHeight() const;

  /** Get center */
  Vec2 getCenter() const;

  /** Get number of tags */
  int getNumTags() const;

  /** Get grid index */
  void getGridIndex(const int tag_id, int &i, int &j) const;

  /** Get object point */
  Vec3 getObjectPoint(const int tag_id, const int corner_index) const;

  /** Get object points */
  std::map<int, Vec3> getObjectPoints() const;
};

} // namespace cartesian
