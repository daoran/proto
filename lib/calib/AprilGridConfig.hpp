#pragma once

namespace xyz {

/** AprilGridConfig **/
struct AprilGridConfig {
  int target_id = 0;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;
  int tag_id_offset = 0;
};

} // namespace xyz
