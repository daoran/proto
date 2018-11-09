#include "prototype/control/att_ctrl.hpp"
#include "prototype/control/carrot_ctrl.hpp"
#include "prototype/control/mission.hpp"
#include "prototype/control/pid.hpp"
#include "prototype/control/pos_ctrl.hpp"
#include "prototype/control/wp_ctrl.hpp"

#include "prototype/core/quaternion/jpl.hpp"
#include "prototype/core/config.hpp"
#include "prototype/core/data.hpp"
#include "prototype/core/euler.hpp"
#include "prototype/core/file.hpp"
#include "prototype/core/gps.hpp"
#include "prototype/core/linalg.hpp"
#include "prototype/core/log.hpp"
#include "prototype/core/math.hpp"
#include "prototype/core/stats.hpp"
#include "prototype/core/time.hpp"

#include "prototype/dataset/euroc/camera_data.hpp"
#include "prototype/dataset/euroc/ground_truth.hpp"
#include "prototype/dataset/euroc/imu_data.hpp"
#include "prototype/dataset/euroc/mav_dataset.hpp"
#include "prototype/dataset/kitti/raw/calib.hpp"
#include "prototype/dataset/kitti/raw/oxts.hpp"
#include "prototype/dataset/kitti/raw/parse.hpp"
#include "prototype/dataset/kitti/raw/raw.hpp"

#include "prototype/driver/camera/camera.hpp"
#include "prototype/driver/gimbal/sbgc.hpp"
#include "prototype/driver/imu/mpu6050.hpp"
#include "prototype/driver/pwm/pca9685.hpp"
#include "prototype/driver/i2c.hpp"
#include "prototype/driver/uart.hpp"

#include "prototype/model/gimbal.hpp"
#include "prototype/model/mav.hpp"
#include "prototype/model/two_wheel.hpp"

#include "prototype/vision/camera/camera_geometry.hpp"
#include "prototype/vision/camera/equi.hpp"
#include "prototype/vision/camera/pinhole.hpp"
#include "prototype/vision/camera/radtan.hpp"
#include "prototype/vision/feature2d/draw.hpp"
#include "prototype/vision/feature2d/grid_fast.hpp"
#include "prototype/vision/feature2d/grid_good.hpp"
#include "prototype/vision/util.hpp"

namespace prototype {

// MACROS
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

// MACROS
#ifndef CHECK
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }
#endif

} //  namespace prototype
