#ifndef TAGDETECTOR_H
#define TAGDETECTOR_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "ethz_apriltag/TagDetection.h"
#include "ethz_apriltag/TagFamily.h"
#include "ethz_apriltag/FloatImage.h"
#include "ethz_apriltag/Tag36h11.h"

namespace ethz_apriltag {

class TagDetector {
public:
  TagFamily thisTagFamily;

  //! Constructor
  // note: TagFamily is instantiated here from TagCodes
  TagDetector(const TagCodes &tagCodes = tagCodes36h11,
              const size_t blackBorder = 2)
      : thisTagFamily(tagCodes, blackBorder) {}

  std::vector<TagDetection> extractTags(const cv::Mat &image);
};

} // namespace

#endif
