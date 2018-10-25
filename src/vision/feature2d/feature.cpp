#include "prototype/vision/feature2d/feature.hpp"

namespace prototype {

Feature::Feature() {}

Feature::Feature(const vec2_t &pt) : kp{cv::Point2f(pt(0), pt(1)), 1.0f} {}

Feature::Feature(const vec2_t &pt, const vec3_t &ground_truth)
    : kp{cv::Point2f(pt(0), pt(1)), 1.0f}, ground_truth{ground_truth} {}

Feature::Feature(const cv::Point2f &pt) : kp{pt, 1.0f} {}

Feature::Feature(const cv::KeyPoint &kp) : kp{kp} {}

Feature::Feature(const cv::KeyPoint &kp, const cv::Mat &desc)
    : kp{kp}, desc{desc} {}

void Feature::setTrackID(const TrackID &track_id) { this->track_id = track_id; }

vec2_t Feature::getKeyPoint() { return vec2_t{this->kp.pt.x, this->kp.pt.y}; }

vec2_t Feature::getKeyPoint() const { return vec2_t{this->kp.pt.x, this->kp.pt.y}; }

std::ostream &operator<<(std::ostream &os, const Feature &f) {
  os << "track_id: " << f.track_id << ", ";
  os << "kp: (" << f.kp.pt.x << ", " << f.kp.pt.y << "), ";
  os << "desc: " << f.desc.size() << std::endl;
  return os;
}

} //  namespace prototype
