#ifndef PROTOTYPE_CALIB_CALIB_CAMERA_NBV_HPP
#define PROTOTYPE_CALIB_CALIB_CAMERA_NBV_HPP

#include <iostream>
#include <string>
#include <memory>

#include <ceres/ceres.h>

#include "prototype/calib/calib_camera.hpp"

namespace proto {

aprilgrid_t
nbv_create_aprilgrid(const calib_target_t &target,
                     const camera_geometry_t<pinhole_t, radtan4_t> &camera,
                     const mat4_t &T_CF);

void nbv_draw_aprilgrid(const aprilgrid_t grid,
                        const pinhole_t &pinhole,
                        const radtan4_t &radtan,
                        const mat4_t &T_CT,
                        cv::Mat &frame);

void nbv_find(const calib_target_t &target,
              const aprilgrids_t &aprilgrids,
              const pinhole_t &pinhole,
              const radtan4_t &radtan,
              mat4_t &nbv_pose,
              aprilgrid_t &nbv_grid);

double calib_camera_nbv_solve(aprilgrids_t &aprilgrids,
                              pinhole_t &pinhole,
                              radtan4_t &radtan,
                              mat4s_t &T_CF);

int calib_camera_nbv(const std::string &target_path,
                     const size_t max_frames = 15);

int calib_camera_batch(const std::string &target_path,
                       const size_t max_frames = 15);

} //  namespace proto
#endif // PROTOTYPE_CALIB_CALIB_CAMERA_NBV_HPP
