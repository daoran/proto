#include "prototype/calib/calib.hpp"
#include "prototype/control/control.hpp"
#include "prototype/core/core.hpp"
#include "prototype/dataset/dataset.hpp"
#include "prototype/driver/driver.hpp"
#include "prototype/estimation/estimation.hpp"
#include "prototype/mav/mav.hpp"
#include "prototype/model/model.hpp"
#include "prototype/vision/vision.hpp"

/*<sidebar_doc>

# [prototype](./)
[[github repo]](https://github.com/chutsu/prototype)

## API
- calib/
    - [aprilgrid.hpp](#calib.aprilgrid)
    - [calib_camera.hpp](#calib.calib_camera)
    - [calib_data.hpp](#calib.calib_data)
    - [calib_gimbal.hpp](#calib.calib_gimbal)
    - [calib_stereo.hpp](#calib.calib_stereo)
- control/
    - [carrot_ctrl.hpp](#control.carrot_ctrl)
    - [pid.hpp](#control.pid)
- core/
    - [config.hpp](#core.config)
    - [data.hpp](#core.data)
    - [file.hpp](#core.file)
    - [gps.hpp](#core.gps)
    - [log.hpp](#core.log)
    - [math.hpp](#core.math)
    - [tf.hpp](#core.tf)
    - [time.hpp](#core.time)
- dataset/
    - [euroc.hpp](#dataset.euroc)
    - [kitti.hpp](#dataset.kitti)
    - [timeline.hpp](#dataset.timeline)
- driver/
    - [camera.hpp](#driver.camera)
    - [mpu6050.hpp](#driver.mpu6050)
    - [pca9685.hpp](#driver.pca9685)
    - [sbgc.hpp](#driver.sbgc)
    - [i2c.cpp](#driver.i2c)
    - [uart.cpp](#driver.uart)
- mav/
    - [att_ctrl.hpp](#mav.att_ctrl)
    - [mission.hpp](#mav.mission)
    - [pos_ctrl.hpp](#mav.pos_ctrl)
    - [wp_ctrl.hpp](#mav.wp_ctrl)
- model/
    - [gimbal.hpp](#model.gimbal)
    - [mav.hpp](#model.mav)
    - [two_wheel.hpp](#model.two_wheel)
- vision/
    - camera/
        - [camera_geometry.hpp](#vision.camera.camera_geometry)
        - [equi.hpp](#vision.camera.equi)
        - [pinhole.hpp](#vision.camera.pinhole)
        - [radtan.hpp](#vision.camera.radtan)
    - feature2d/
        - [draw.hpp](#vision.feature2d.draw)
        - [grid_fast.hpp](#vision.feature2d.grid_fast)
        - [grid_good.hpp](#vision.feature2d.grid_good)

<sidebar_doc>*/
