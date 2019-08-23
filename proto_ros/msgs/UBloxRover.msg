# UBX-NAV-PVT
uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code
               # differential, 5: Real-Time Kinematic, float, 6: Real-Time
               # Kinematic, fixed, 8: Extrapolated. Some applications will
               # not use the value of this field unless it is at least
               # two, so always correctly fill in the fix.
uint8 vel_ned_valid # True if NED velocity is valid.
uint64 timestamp # [us] Timestamp (synced to UNIX time or since system boot).
uint64 time_utc_usec # [us] Timestamp (microseconds, UTC), this is the
                     # timestamp which comes from the gps module. It might
                     # be unavailable right after cold start, indicated by
                     # a value of 0
int32 timestamp_time_relative # timestamp + timestamp_time_relative =
                              # Time of the UTC timestamp since system
                              # start, (microseconds)
int32 lat # Latitude in 1E-7 degrees.
int32 lon # Longitude in 1E-7 degrees.
int32 alt # Altitude in 1E-3 meters above MSL, (millimetres).
int32 alt_ellipsoid # Altitude in 1E-3 meters above Ellipsoid, (millimetres).
float32 s_variance_m_s #[m/s] GPS speed accuracy estimate, (metres/sec).
float32 eph # [m] GPS horizontal position accuracy (metres).
float32 epv # [m] GPS vertical position accuracy (metres).
uint8 satellites_used # Number of satellites used .

# UBX-NAV-VELNED
float32 vel_m_s # [m/s] GPS ground speed, (metres/sec).
float32 vel_n_m_s # [m/s] GPS North velocity, (metres/sec).
float32 vel_e_m_s # [m/s] GPS East velocity, (metres/sec).
float32 vel_d_m_s # [m/s] GPS Down velocity, (metres/sec).
float32 cog_rad # [rad] Course over ground (NOT heading, but direction of
                # movement), -PI..PI, (radians) .
float32 c_variance_rad # [rad] GPS course accuracy estimate, (radians).

# UBX-NAV-DOP
float32 hdop # Horizontal dilution of precision.
float32 vdop # Vertical dilution of precision.

# UBX-MON-RF
int32 noise_per_ms # GPS noise per millisecond.
int32 jamming_indicator # indicates jamming is occurring.

# UBX-NAV-EOE
float32 heading # [rad] heading in NED. Set to NaN if not set (used for dual
                # antenna GPS), (rad, [-PI, PI]).
