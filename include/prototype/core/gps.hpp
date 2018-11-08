#ifndef PROTOTYPE_CORE_GPS_HPP
#define PROTOTYPE_CORE_GPS_HPP

#include <cmath>

#include "prototype/core/math.hpp"

namespace prototype {

#define EARTH_RADIUS_M 6378137.0

/**
 * Calculate new latitude and logitude coordinates with an offset in North and
 * East direction
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param offset_N Offset in North direction (meters)
 * @param offset_E Offset in East direction (meters)
 * @param lat_new New latitude (decimal format)
 * @param lon_new New longitude (decimal format)
 */
void latlon_offset(double lat_ref,
                   double lon_ref,
                   double offset_N,
                   double offset_E,
                   double *lat_new,
                   double *lon_new);

/**
 * Calculate difference in distance in North and East from two GPS coordinates
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param lat Latitude of point of interest (decimal format)
 * @param lon Longitude of point of interest (decimal format)
 * @param dist_N Distance of point of interest in North axis [m]
 * @param dist_E Distance of point of interest in East axis [m]
 */
void latlon_diff(double lat_ref,
                 double lon_ref,
                 double lat,
                 double lon,
                 double *dist_N,
                 double *dist_E);

/**
 * Calculate Euclidean distance between two GPS coordintes
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param lat Latitude of point of interest (decimal format)
 * @param lon Longitude of point of interest (decimal format)
 *
 * @returns Euclidean distance between two GPS coordinates [m]
 */
double latlon_dist(double lat_ref, double lon_ref, double lat, double lon);

} // namespace prototype
#endif // PROTOTYPE_CORE_GPS_HPP
