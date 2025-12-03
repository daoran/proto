#pragma once
#include "../Core.hpp"

namespace xyz {

/** Calibration Target Geometry */
class CalibTargetGeometry {
private:
  int target_id_;
  Vec7 target_pose_;
  std::map<int, Vec3> target_points_;

public:
  CalibTargetGeometry() = delete;
  CalibTargetGeometry(const int target_id, const Vec7 &target_pose);
  virtual ~CalibTargetGeometry() = default;

  /** Get target pose **/
  Vec7 getPose() const;

  /** Get target pose pointer **/
  double *getPosePtr();

  /** Get target points **/
  std::map<int, Vec3> &getPoints();

  /** Get target point **/
  Vec3 &getPoint(const int point_id);

  /** Get target point pointer **/
  double *getPointPtr(const int point_id);

  /** Add target point **/
  void addPoint(const int point_id, const Vec3 &point);
};

} // namespace xyz
