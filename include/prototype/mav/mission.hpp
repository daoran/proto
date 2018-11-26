#ifndef PROTOTYPE_MAV_MISSION_HPP
#define PROTOTYPE_MAV_MISSION_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <vector>

#include "prototype/core/core.hpp"

namespace prototype {

// ERROR MESSAGES
#define EDISTLATLON "Waypoint %d: (%f, %f) has dist > %f from prev waypoint!"
#define EINVLATLON "Invalid latlon (%f, %f)!"
#define EINVALT "Invalid altitude %f!"
#define EINVSTAY "Invalid staytime %f!"
#define EINVHEADING                                                            \
  "Invalid heading %f! Should be between -180.0 to 180.0 degrees"

// CONSTANTS
#define GPS_WAYPOINTS 0
#define LOCAL_WAYPOINTS 1
#define CTRACK_HORIZ 0
#define CTRACK_3D 1

/**
 * Waypoint
 */
struct waypoint_t {
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double staytime = 0.0;
  double heading = 0.0;
};

/**
 * Setup waypoint with `latitude` and `longitude`.
 *
 * @returns Waypoint
 */
waypoint_t waypoint_setup(const double latitude, const double longitude);

/**
 * Setup waypoint.
 *
 * @returns Waypoint
 */
waypoint_t waypoint_setup(const double latitude,
                          const double longitude,
                          const double altitude,
                          const double staytime,
                          const double heading);

/**
 * Calculate distance between waypoint `wp_a` and `wp_b` in [m].
 * @returns Distance between waypoint `wp_a` and waypoint `wp_b`
 */
double waypoint_distance(const waypoint_t &wp_a, const waypoint_t &wp_b);

/**
 * `waypoint_t` to output stream
 */
std::ostream &operator<<(std::ostream &out, const waypoint_t &wp);

/**
 * Mission
 */
struct wp_mission_t {
  bool configured = false;
  bool completed = false;

  double home_lat = 0.0;
  double home_lon = 0.0;
  double home_alt = 0.0;

  bool check_waypoints = true;
  double threshold_waypoint_gap = 20.0;
  double threshold_waypoint_reached = 0.2;
  double desired_velocity = 0.5;
  double look_ahead_dist = 0.5;

  vec3s_t gps_waypoints;
  vec3s_t local_waypoints;
  int waypoint_index = 0;
  vec3_t wp_start = vec3_t::Zero();
  vec3_t wp_end = vec3_t::Zero();
};

/**
 * Configure mission with configuration file `config_file`.
 *
 * @returns 0 for success, -1 for failure to load / parse config file, and -2
 * for invalid GPS waypoints
 */
int wp_mission_configure(wp_mission_t &m, const std::string &config_file);

/**
 * Load GPS waypoints with the list of waypoints `wps`, where waypoint type is
 * denoted by `type`.
 *
 * @returns 0 for success, -1 for failure
 */
int wp_mission_load_waypoints(wp_mission_t &m,
                              const std::vector<double> &wps,
                              const int type);

/**
 * Check GPS waypoints to make sure the distance between waypoints does not
 * exceed `mission_t.waypoint_threshold`.
 *
 * @returns 0 for success, -1 for no waypoints loaded, and finally -2 for
 * invalid GPS waypoints.
 */
int wp_mission_check_gps_waypoints(wp_mission_t &m);

/**
 * Set GPS home point latitude `home_lat`, longitude `home_lon`, and calculate
 * local waypoints by converting GPS to local waypoints.
 *
 * @returns 0 for success, -1 for failure.
 */
int wp_mission_set_gps_homepoint(wp_mission_t &m,
                                 const double home_lat,
                                 const double home_lon);

/**
 * Calculate closest point.
 *
 * @returns Closest point
 */
vec3_t wp_mission_closest_point(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Calcuate which side the point is compared to waypoint track.
 *
 * @returns 0 if position is colinear with waypoint track, 1 if position is
 * left of waypoint track, and -1 if position is right of waypoint track.
 */
int wp_mission_point_line_side(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Calcuate crosstrack error.
 *
 * @returns Cross track error.
 */
double wp_mission_crosstrack_error(const wp_mission_t &m,
                                   const vec3_t &p_G,
                                   int mode = CTRACK_HORIZ);

/**
 * Calculate waypoint yaw. This function assumes we are operating in the NWU
 * frame, where a 0 heading starts from the x-axis and goes counter-clock-wise.
 *
 * @returns Waypoint yaw
 */
double wp_mission_waypoint_heading(const wp_mission_t &m);

/**
 * Interpolate mission waypoint point.
 *
 * @returns Interpolated mission waypoint.
 */
vec3_t wp_mission_waypoint_interpolate(const wp_mission_t &m,
                                       const vec3_t &p_G,
                                       const double r);

/**
 * Check whether waypoint is reached.
 *
 * @returns 0 if waypoint not reached, 1 if waypoint is reached, and -1 if
 * mission is not configured.
 */
int wp_mission_waypoint_reached(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Update mission with current position in the global frame `p_G` and output
 * interpolated `waypoint`.
 *
 * @returns 0 for success, -1 if mission is not configured, and -2 for no more
 * waypoints.
 */
int wp_mission_update(wp_mission_t &m, const vec3_t &p_G, vec3_t &waypoint);

} //  namespace prototype
#endif // PROTOTYPE_MAV_MISSION_HPP
