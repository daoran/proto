/**
 * @file
 * @ingroup feature2d
 */
#ifndef PROTOTYPE_VISION_FEATURE2D_FEATURE_TRACK_HPP
#define PROTOTYPE_VISION_FEATURE2D_FEATURE_TRACK_HPP

#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "prototype/core.hpp"
#include "prototype/vision/feature2d/feature.hpp"
#include "prototype/model/gimbal.hpp"

namespace prototype {
/**
 * @addtogroup feature2d
 * @{
 */

// Track ID
using FrameID = long int;

// Feature track type
#define MONO_TRACK 0
#define STATIC_STEREO_TRACK 1
#define DYNAMIC_STEREO_TRACK 2

/**
 * Feature track
 */
struct FeatureTrack {
  int type = -1;

  // General
  TrackID track_id = -1;
  FrameID frame_start = -1;
  FrameID frame_end = -1;
  TrackID related = -1;
  size_t min_track_length = -1;

  // Monocular camera
  Features track;

  // Stereo camera
  Features track0;
  Features track1;
  // -- Static track
  mat4_t T_cam1_cam0;
  // -- Dynamic track
  std::vector<vec2_t> joint_angles;

  // 3D position of feature relative to camera's first pose
  vec3_t p_C0_f = vec3_t::Zero();

  FeatureTrack();

  // Monocular Feature Track
  FeatureTrack(const TrackID &track_id,
               const FrameID &frame_id,
               const Feature &f1,
               const Feature &f2);

  // Static Stereo Feature Track
  FeatureTrack(const TrackID &track_id,
               const FrameID &frame_start,
               const FrameID &frame_end,
               const Features &tracks0,
               const Features &tracks1,
               const mat4_t &T_cam1_cam0);

  // Dynamic Stereo Feature Track
  FeatureTrack(const TrackID &track_id,
               const FrameID &frame_start,
               const FrameID &frame_end,
               const Features &track0,
               const Features &track1,
               const vec2_t &joint_angles_t0,
               const vec2_t &joint_angles_t1);

  /**
   * Update feature track
   *
   * @param frame_id Frame ID
   * @param data Feature
   */
  void update(const FrameID &frame_id, const Feature &data);

  /**
   * Update static stereo feature track
   *
   * @param frame_id Frame ID
   * @param cam0_f Feature from camera 0
   * @param cam1_f Feature from camera 1
   */
  void updateStereo(const FrameID &frame_id,
                    const Feature &cam0_f,
                    const Feature &cam1_f);

  /**
   * Update dynamic stereo feature track
   *
   * @param frame_id Frame ID
   * @param cam0_f Feature from camera 0
   * @param cam1_f Feature from camera 1
   * @param joint_angles Gimbal joint angles
   */
  void updateStereo(const FrameID &frame_id,
                    const Feature &cam0_f,
                    const Feature &cam1_f,
                    const vec2_t &joint_angles);

  /**
   * Slice feature track
   *
   * @param frame_start New frame start
   * @param frame_end New frame end
   */
  void slice(const size_t frame_start, const size_t frame_end);

  /**
   * Return last feature seen
   *
   * @returns Last feature
   */
  Feature &last();

  /**
   * Return feature track length
   *
   * @returns Size of feature track
   */
  size_t trackedLength();

  /**
   * Return feature track length
   *
   * @returns Size of feature track
   */
  size_t trackedLength() const;
};

/**
  * FeatureTrack to string
  */
std::ostream &operator<<(std::ostream &os, const FeatureTrack &track);

/**
 * Feature tracks
 */
using FeatureTracks = std::vector<FeatureTrack>;

/**
 * Feature track to CSV file
 *
 * @param track Feature track
 * @param output_path Output path
 */
int save_feature_track(const FeatureTrack &track,
                       const std::string &output_path);

/**
 * Feature tracks to CSV file
 *
 * @param tracks Feature track
 * @param output_path Output path
 */
int save_feature_tracks(const FeatureTracks &tracks,
                        const std::string &output_path);

/** @} group feature2d */
} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_FEATURE_TRACK_HPP
