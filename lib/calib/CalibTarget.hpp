#pragma once

#include "../Core.hpp"

namespace xyz {

/** Calibration Target */
class CalibTarget {
protected:
  timestamp_t ts_ = 0;
  int camera_id_ = 0;
  std::string target_type_;
  int target_id_ = 0;
  int tag_rows_ = 0;
  int tag_cols_ = 0;
  double tag_size_ = 0.0;
  double tag_spacing_ = 0.0;

  // Data
  struct TagDetection {
    std::set<int> corner_indicies;
    std::unordered_map<int, Vec2> keypoints;
  };
  std::unordered_map<int, TagDetection> data_;

public:
  CalibTarget(const timestamp_t &timestamp,
              const int camera_id,
              const std::string &target_type,
              const int target_id,
              const int tag_rows,
              const int tag_cols,
              const double tag_size,
              const double tag_spacing);
  virtual ~CalibTarget() = default;

  /** Check if detected */
  bool detected() const;

  /** Get Timestamp **/
  timestamp_t getTimestamp() const;

  /** Get Camera ID **/
  int getCameraId() const;

  /** Get Target Type **/
  std::string getTargetType() const;

  /** Get Target ID **/
  int getTargetId() const;

  /** Return number of tag rows */
  int getTagRows() const;

  /** Return number of tag cols */
  int getTagCols() const;

  /** Return number of tag size */
  double getTagSize() const;

  /** Return number of tag spacing */
  double getTagSpacing() const;

  /** Get the 2D center of the AprilGrid */
  Vec2 getCenter2d() const;

  /** Get the 3D center of the AprilGrid */
  Vec3 getCenter3d() const;

  /** Get number detected */
  int getNumDetected() const;

  /** Get measurements **/
  virtual void getMeasurements(std::vector<int> &tag_ids,
                               std::vector<int> &corner_indicies,
                               Vec2s &keypoints,
                               Vec3s &object_points) const = 0;

  /** Check to see if AprilGrid has specific tag id and corner index */
  bool has(const int tag_id, const int corner_index) const;

  /** Add measurmeent */
  void add(const int tag_id, const int corner_index, const Vec2 &kp);

  /** Remove measurmeent */
  void remove(const int tag_id, const int corner_index);

  /** Remove measurmeent */
  void remove(const int tag_id);

  /** Draw AprilGrid */
  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;

  /** Imshow AprilGrid */
  void imshow(const std::string &title, const cv::Mat &image) const;
};

} // namespace xyz
