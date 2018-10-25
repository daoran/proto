#include "prototype/control/mission.hpp"

namespace prototype {

waypoint_t waypoint_setup(const double latitude, const double longitude) {
  waypoint_t wp;
  wp.latitude = latitude;
  wp.longitude = longitude;
  return wp;
}

waypoint_t waypoint_setup(const double latitude,
                          const double longitude,
                          const double altitude,
                          const double staytime,
                          const double heading) {
  waypoint_t wp;
  wp.latitude = latitude;
  wp.longitude = longitude;
  wp.altitude = altitude;
  wp.staytime = staytime;
  wp.heading = heading;
  return wp;
}

double waypoint_distance(const waypoint_t &wp_a,
                         const waypoint_t &wp_b) {
  return latlon_dist(wp_a.latitude, wp_a.longitude,
                     wp_b.latitude, wp_b.longitude);
}

std::ostream &operator<<(std::ostream &out, const waypoint_t &wp) {
  out << "latitude: " << wp.latitude << std::endl;
  out << "longitude: " << wp.longitude << std::endl;
  out << "altitude: " << wp.altitude << std::endl;
  out << "staytime: " << wp.staytime << std::endl;
  out << "heading: " << wp.heading;
  return out;
}

int wp_mission_configure(wp_mission_t &m, const std::string &config_file) {
  ConfigParser parser;
  std::vector<double> wp_data;
  std::string wp_type;

  // Load config
  // clang-format off
  parser.addParam("desired_velocity", &m.desired_velocity);
  parser.addParam("look_ahead_dist", &m.look_ahead_dist);
  parser.addParam("threshold_waypoint_gap", &m.threshold_waypoint_gap);
  parser.addParam("threshold_waypoint_reached", &m.threshold_waypoint_reached);
  parser.addParam("waypoint_type", &wp_type);
  parser.addParam("waypoints", &wp_data);
  // clang-format on
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // Check number waypoint data
  if (wp_data.size() % 3 != 0) {
    LOG_ERROR("Invalid number of waypoint data!");
    return -1;
  }

  // Load waypoints
  if (wp_type == "GPS" && wp_mission_load_waypoints(m, wp_data, GPS_WAYPOINTS) != 0) {
    LOG_ERROR("Failed to load GPS waypoints!");
    return -1;
  } else if (wp_type == "LOCAL" && wp_mission_load_waypoints(m, wp_data, LOCAL_WAYPOINTS) != 0) {
    LOG_ERROR("Failed to load local waypoints!");
    return -1;
  } else if (wp_type != "GPS" && wp_type != "LOCAL") {
    LOG_ERROR("Invalid waypoint type [%s]!", wp_type.c_str());
    return -1;
  }

  // Update
  m.configured = true;
  return 0;
}

int wp_mission_load_waypoints(wp_mission_t &m,
                              const std::vector<double> &wps,
                              const int type) {
  if (type == GPS_WAYPOINTS) {
    // Convert waypoint data into waypoints in the local frame
    for (size_t i = 0; i < wps.size(); i += 3) {
      const double lat = wps[i];
      const double lon = wps[i + 1];
      const double alt = wps[i + 2];

      // Check lat, lon
      if (fltcmp(lat, 0.0) == 0.0 || fltcmp(lon, 0.0) == 0.0) {
        LOG_ERROR(EINVLATLON, lat, lon);
        return -1;
      }

      // Check alt
      if (fltcmp(alt, 0.0) == 0.0) {
        LOG_ERROR(EINVALT, alt);
        return -1;
      }

      m.gps_waypoints.emplace_back(lat, lon, alt);
    }

    // Check waypoints
    if (m.check_waypoints && wp_mission_check_gps_waypoints(m) != 0) {
      return -2;
    }

  } else if (type == LOCAL_WAYPOINTS) {
    // Load local waypoints
    for (size_t i = 0; i < wps.size(); i += 3) {
      const vec3_t wp{wps[i], wps[i + 1], wps[i + 2]};
      std::cout << "Adding local waypoint: " << wp.transpose() << std::endl;
      m.local_waypoints.emplace_back(wp);
    }

    // Set first pair of waypoints
    m.wp_start = m.local_waypoints[0];
    m.wp_end = m.local_waypoints[1];

  } else {
    return -3;
  }

  return 0;
}

int wp_mission_check_gps_waypoints(wp_mission_t &m) {
  // Pre-check
  if (m.gps_waypoints.size() <= 2) {
    return -1;
  }

  // Check waypoint gaps
  vec3_t last_wp = m.gps_waypoints.front();
  for (size_t i = 1; i < m.gps_waypoints.size(); i++) {
    // Calculate distance between current and last waypoint
    vec3_t wp = m.gps_waypoints[i];

    // Check distance
    double dist = latlon_dist(last_wp(0), last_wp(1), wp(0), wp(1));
    if (dist > m.threshold_waypoint_gap) {
      LOG_ERROR(EDISTLATLON,
                (int) i + 1,
                wp(0),
                wp(1),
                m.threshold_waypoint_gap);
      return -2;
    }

    // Update last waypoint
    last_wp = wp;
  }

  return 0;
}

int wp_mission_set_gps_homepoint(wp_mission_t &m,
                                 const double home_lat,
                                 const double home_lon) {
  // Pre-check
  if (m.gps_waypoints.size() == 0) {
    return -1;
  }

  // Convert
  for (auto gps : m.gps_waypoints) {
    // Convert lat lon to local frame
    const double lat = gps(0);
    const double lon = gps(1);
    const double alt = gps(2);
    double dist_N, dist_E;
    latlon_diff(home_lat, home_lon, lat, lon, &dist_N, &dist_E);

    // Add to local waypoints in NWU
    const vec3_t nwu{dist_N, -1.0 * dist_E, alt};
    std::cout << "Adding local waypoint (nwu): " << nwu.transpose();
    std::cout << std::endl;
    m.local_waypoints.push_back(nwu);
  }

  // Set first pair of waypoints
  m.wp_start = m.local_waypoints[0];
  m.wp_end = m.local_waypoints[1];

  return 0;
}

vec3_t wp_mission_closest_point(const wp_mission_t &m, const vec3_t &p_G) {
  // Calculate closest point
  const vec3_t v1 = p_G - m.wp_start;
  const vec3_t v2 = m.wp_end - m.wp_start;
  const double t = v1.dot(v2) / v2.squaredNorm();

  // Make sure the point is between wp_start and wp_end
  if (t < 0) {
    return m.wp_start;
  } else if (t > 1) {
    return m.wp_end;
  }

  return m.wp_start + t * v2;
}

int wp_mission_point_line_side(const wp_mission_t &m, const vec3_t &p_G) {
  const vec3_t a = m.wp_start;
  const vec3_t b = m.wp_end;
  const vec3_t c = p_G;
  const double s =
      ((b(0) - a(0)) * (c(1) - a(1)) - (b(1) - a(1)) * (c(0) - a(0)));

  // Position is colinear with waypoint track
  if (fltcmp(s, 0.0) == 0) {
    return 0;
  }

  // Position is left of waypoint track
  if (s > 0.0) {
    return 1;
  }

  // Position is right of waypoint track
  return -1;
}

double wp_mission_crosstrack_error(const wp_mission_t &m,
                                   const vec3_t &p_G,
                                   int mode) {
  vec3_t BA = m.wp_start - p_G;
  vec3_t BC = m.wp_start - m.wp_end;

  // Only calculate horizontal crosstrack error by setting z to 0
  if (mode == CTRACK_HORIZ) {
    BA(2) = 0.0;
    BC(2) = 0.0;
  }

  // Crosstrack error
  const double error = (BA.cross(BC)).norm() / BC.norm();

  // Check which side the point is on
  const int side = wp_mission_point_line_side(m, p_G);

  return error * side;
}

double wp_mission_waypoint_heading(const wp_mission_t &m) {
  const double dx = m.wp_end(0) - m.wp_start(0);
  const double dy = m.wp_end(1) - m.wp_start(1);

  // Calculate heading
  double heading = atan2(dy, dx);
  if (heading > M_PI) {
    heading -= 2 * M_PI;
  } else if (heading < -M_PI) {
    heading += 2 * M_PI;
  }

  return heading;
}

vec3_t wp_mission_waypoint_interpolate(const wp_mission_t &m,
                                       const vec3_t &p_G,
                                       const double r) {
  // Get closest point
  const vec3_t pt_on_line = wp_mission_closest_point(m, p_G);

  // Calculate waypoint between wp_start and wp_end
  const vec3_t v = m.wp_end - m.wp_start;
  const vec3_t u = v / v.norm();
  return pt_on_line + r * u;
}

int wp_mission_waypoint_reached(const wp_mission_t &m, const vec3_t &p_G) {
  // Pre-check
  if (m.configured == false) {
    return -1;
  }

  // Calculate distance to waypoint
  const vec3_t x = m.wp_end - p_G;
  const double dist = x.norm();

  // Waypoint reached?
  if (dist > m.threshold_waypoint_reached) {
    return 0;
  } else {
    LOG_INFO("Waypoint (%f, %f, %f) reached!",
             m.wp_end(0),
             m.wp_end(1),
             m.wp_end(2));
    return 1;
  }
}

int wp_mission_update(wp_mission_t &m, const vec3_t &p_G, vec3_t &waypoint) {
  // Pre-check
  if (m.configured == false) {
    return -1;
  } else if (m.local_waypoints.size() == 0) {
    return -3;
  }

  // Interpolate new waypoint
  if (m.completed != true) {
    waypoint = wp_mission_waypoint_interpolate(m, p_G, m.look_ahead_dist);
  }

  // Waypoint reached? get new wp_start and wp_end
  if (wp_mission_waypoint_reached(m, waypoint)) {
    if ((m.waypoint_index + 2) == (int) m.local_waypoints.size()) {
      m.completed = true;
      m.waypoint_index = 0;
      return -2;

    } else {
      m.wp_start = m.local_waypoints[m.waypoint_index + 1];
      m.wp_end = m.local_waypoints[m.waypoint_index + 2];
      m.waypoint_index++;
    }
  }

  return 0;
}

} //  namespace prototype
