#ifndef PROTOTYPE_CONTROL_MISSION_HPP
#define PROTOTYPE_CONTROL_MISSION_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <vector>

#include "prototype/core.hpp"

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
 * Setup waypoint
 *
 * @param[in] latitude Latitude
 * @param[in] longitude Longitude
 * @returns Waypoint
 */
waypoint_t waypoint_setup(const double latitude, const double longitude);

/**
 * Setup waypoint
 *
 * @param[in] latitude Latitude
 * @param[in] longitude Longitude
 * @param[in] altitude Altitude
 * @param[in] staytime Staytime
 * @param[in] heading Heading
 *
 * @returns Waypoint
 */
waypoint_t waypoint_setup(const double latitude,
                          const double longitude,
                          const double altitude,
                          const double staytime,
                          const double heading);

/**
 * Calculate distance away from another waypoint
 *
 * @param[in] wp_a 1st Waypoint to calculate distance away from
 * @param[in] wp_b 2nd Waypoint to calculate distance away to
 * @return Distance between waypoint `wp_a` and waypoint `wp_b`
 */
double waypoint_distance(const waypoint_t &wp_a, const waypoint_t &wp_b);

/**
 * Waypoint to output stream
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

  std::vector<vec3_t> gps_waypoints;
  std::vector<vec3_t> local_waypoints;
  int waypoint_index = 0;
  vec3_t wp_start = vec3_t::Zero();
  vec3_t wp_end = vec3_t::Zero();
};

/**
 * Configure
 *
 * @param[in,out] m Mission
 * @param[in] config_file Path to configuration fileN
 * @return
 *    - 0: success
 *    - -1: failure to load / parse configuration file
 *    - -2: invalid GPS waypoints
 */
int wp_mission_configure(wp_mission_t &m, const std::string &config_file);

/**
 * Load GPS waypoints
 *
 * @param[in,out] m Mission
 * @param[in] wps Waypoints
 * @param[in] type Type
 * @return 0 for success, -1 for failure
 */
int wp_mission_load_waypoints(wp_mission_t &m,
                              const std::vector<double> &wps,
                              const int type);

/**
 * Check GPS waypoints
 *
 * Make sure the distance between waypoints does not exceed
 * `Mission.waypoint_threshold`
 *
 * @param[in,out] m Mission
 * @return
 *    - 0: success
 *    - -1: no waypoints loaded
 *    - -2: invalid GPS waypoints
 */
int wp_mission_check_gps_waypoints(wp_mission_t &m);

/**
 * Set GPS home point and calculate local waypoints by converting GPS to
 * local waypoints
 *
 * @param[in,out] m Mission
 * @param[in] home_lat Home latitude point
 * @param[in] home_lon Home longitude point
 * @return 0 for success, -1 for failure
 */
int wp_mission_set_gps_homepoint(wp_mission_t &m,
                                 const double home_lat,
                                 const double home_lon);

/**
 * Calculate closest point
 *
 * @param[in] m Mission
 * @param[in] p_G Position in global frame
 * @return Closest point
 */
vec3_t wp_mission_closest_point(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Calcuate which side the point is compared to waypoint track
 *
 * @param[in] m Mission
 * @param[in] p_G Position
 * @return
 *    - 0: Position is colinear with waypoint track
 *    - 1: Position is left of waypoint track
 *    - -1: Position is right of waypoint track
 */
int wp_mission_point_line_side(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Calcuate crosstrack error
 *
 * @param[in] m Mission
 * @param[in] p_G Position
 * @param[in] mode Cross track error mode
 * @return Cross track error
 */
double wp_mission_crosstrack_error(const wp_mission_t &m,
                                   const vec3_t &p_G,
                                   int mode = CTRACK_HORIZ);

/**
 * Calculate waypoint yaw
 *
 * This function assumes we are operating in the NWU frame, where a 0 heading
 * starts from the x-axis and goes counter-clock-wise.
 *
 * @param[in] m Mission
 * @return Waypoint yaw
 */
double wp_mission_waypoint_heading(const wp_mission_t &m);

/**
 * Calculate waypoint point
 *
 * @param[in] m Mission
 * @param[in] p_G Position in global frame
 * @param[in] r Lookahead distance in meters
 */
vec3_t wp_mission_waypoint_interpolate(const wp_mission_t &m,
                                       const vec3_t &p_G,
                                       const double r);

/**
 * Check whether waypoint is reached
 *
 * @param[in] m Mission
 * @param[in] p_G Position in global frame
 * @return
 *    - 0: Waypoint not reached
 *    - 1: Waypoint reached
 *    - -1: Not configured
 */
int wp_mission_waypoint_reached(const wp_mission_t &m, const vec3_t &p_G);

/**
 * Update waypoint
 *
 * @param[in,out] m Mission
 * @param[in] p_G Position in global frame
 * @param[out] waypoint Waypoint to update
 * @return
 *   - 0: Success
 *   - -1: Not configured
 *   - -2: No more waypoints
 */
int wp_mission_update(wp_mission_t &m, const vec3_t &p_G, vec3_t &waypoint);

} //  namespace prototype
#endif // PROTOTYPE_CONTROL_MISSION_HPP
