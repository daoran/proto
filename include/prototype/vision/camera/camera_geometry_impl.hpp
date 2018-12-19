#ifndef PROTOTYPE_CALIB_CAMERA_CAMERA_GEOMETRY_IMPL_HPP
#define PROTOTYPE_CALIB_CAMERA_CAMERA_GEOMETRY_IMPL_HPP

namespace proto {

template <typename CM, typename DM>
camera_geometry_t<CM, DM>::camera_geometry_t(const CM &camera_model_,
                                             const DM &distortion_model_)
    : camera_model{camera_model_}, distortion_model{distortion_model_} {}

template <typename CM, typename DM>
camera_geometry_t<CM, DM>::~camera_geometry_t() {}

template <typename CM, typename DM>
vec2_t camera_geometry_project(const camera_geometry_t<CM, DM> &cam,
                               const vec3_t &point) {
  const vec2_t p{point(0) / point(2), point(1) / point(2)};
  const vec2_t point_distorted = distort(cam.distortion_model, p);
  return project(cam.camera_model, point_distorted);
}

} //  namespace proto
#endif // PROTOTYPE_CALIB_CAMERA_CAMERA_GEOMETRY_IMPL_HPP
