#pragma once
#include "../Core.hpp"

namespace xyz {

// Forward declaration
class CalibTargetGeometry;
using CalibTargetGeometryPtr = std::shared_ptr<CalibTargetGeometry>;

/** Calibration Target Geometry */
class CalibTargetGeometry {
private:
  int target_id_;
  Vec7 extrinsic_;
  std::map<int, Vec3> target_points_;

public:
  CalibTargetGeometry() = delete;
  CalibTargetGeometry(const int target_id,
                      const Vec7 &extrinsic,
                      const std::map<int, Vec3> &target_points);
  virtual ~CalibTargetGeometry() = default;

  /** Get target id **/
  int getTargetId() const;

  /** Get target pose **/
  Vec7 getExtrinsic() const;

  /** Get target pose pointer **/
  double *getExtrinsicPtr();

  /** Get transform T_body_camera **/
  Mat4 getTransform() const;

  /** Get target points **/
  std::map<int, Vec3> &getPoints();

  /** Get target point **/
  Vec3 &getPoint(const int point_id);

  /** Get target point pointer **/
  double *getPointPtr(const int point_id);

  /** Add target point **/
  void addPoint(const int point_id, const Vec3 &point);

  /** Set extrinsic */
  void setExtrinsic(const Mat4 &extrinsic);
};

} // namespace xyz
