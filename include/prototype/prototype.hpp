#include "prototype/control/control.hpp"
#include "prototype/core/core.hpp"
#include "prototype/dataset/dataset.hpp"
#include "prototype/driver/driver.hpp"
#include "prototype/model/model.hpp"

/*<sidebar_doc>

# [prototype](./)
[[github repo]](https://github.com/chutsu/prototype)

## API
- calib/
    - [aprilgrid.hpp](#calib.aprilgrid)
    - [chessboard.hpp](#calib.chessboard)
    - [calib_camera.hpp](#calib.calib_camera)
    - [calib_data.hpp](#calib.calib_data)
    - [calib_gimbal.hpp](#calib.calib_gimbal)
- control/
    - [att_ctrl.hpp](#control.att_ctrl)
    - [carrot_ctrl.hpp](#control.carrot_ctrl)
    - [mission.hpp](#control.mission)
    - [pid.hpp](#control.pid)
    - [pos_ctrl.hpp](#control.pos_ctrl)
    - [wp_ctrl.hpp](#control.wp_ctrl)
- core/
    - [config.hpp](#core.config)
    - [data.hpp](#core.data)
    - [euler.hpp](#core.euler)
    - [file.hpp](#core.file)
    - [gps.hpp](#core.gps)
    - [jpl.hpp](#core.jpl)
    - [linalg.hpp](#core.linalg)
    - [log.hpp](#core.log)
    - [math.hpp](#core.math)
    - [stats.hpp](#core.stats)
    - [time.hpp](#core.time)
- dataset/
    - [euroc.hpp](#dataset.euroc)
    - [kitti.hpp](#dataset.kitti)
- driver/
    - camera/
        - [camera.hpp](#driver.camera.camera)
    - gimbal/
        - [sbgc.hpp](#driver.gimbal.sbgc)
    - imu/
        - [mpu6050.hpp](#driver.imu.mpu6050)
    - pwm/
        - [pca9685.hpp](#driver.pca9685)
    - [i2c.cpp](#driver.i2c)
    - [uart.cpp](#driver.uart)
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
    - [util.hpp](#vision.util)

<sidebar_doc>*/
