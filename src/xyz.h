#ifndef XYZ_H
#define XYZ_H

#ifdef __cplusplus
extern "C" {
#endif

// SETTINGS
#define MAX_LINE_LENGTH 9046
#define USE_CERES
// #define USE_STB
// #define USE_GUI
// #define USE_APRILGRID


#include <errno.h>

#ifdef USE_CERES
#include "xyz_ceres_bridge.h"
#endif

#ifdef USE_STB
#include <stb_image.h>
#endif

#ifdef USE_GUI
#include "gui.h"
#endif

#ifdef USE_APRILGRID
#include "xyz_aprilgrid.h"
#endif

#include "macros.h"
#include "logging.h"
#include "time.h"
#include "data.h"
#include "fs.h"
#include "math.h"
#include "cv.h"
#include "state_estimation.h"
#include "dataset.h"
// #include "sim.h"


#ifdef __cplusplus
} // extern C
#endif
#endif // XYZ_H
