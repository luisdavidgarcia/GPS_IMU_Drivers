#ifndef GTU7_CONSTANTS_HPP
#define GTU7_CONSTANTS_HPP

#include <cstdint>

constexpr int default_timeout = 1000;

/** i2c specifics */
constexpr uint8_t uart_port = 0x01;

constexpr int data_stream_register = 0xff;

constexpr int available_bytes_msb = 0xfd;
constexpr int available_bytes_lsb = 0xfe;

constexpr int default_timeout_mills = 2000;
constexpr int default_update_mills = 1000;
constexpr int default_send_rate = 0x01;
constexpr int default_interval_mills = 50;
constexpr bool default_polling_state = false;
constexpr int default_year = -1;

constexpr int valid_date_flag = 0x01;
constexpr int valid_time_flag = 0x02;
constexpr int fully_resolved_flag = 0x04;
constexpr int valid_mag_flag = 0x08;
constexpr int invalid_sync1_flag = 0xff;

/** sam-m8q limits */
constexpr int max_month = 12;
constexpr int min_month = 1;
constexpr int max_day = 31;
constexpr int min_day = 1;
constexpr int max_hour = 23;
constexpr int min_hour = 0;
constexpr int max_minute = 59;
constexpr int min_minute = 0;
constexpr int max_second = 59;
constexpr int min_second = 0;
constexpr float max_longtitude = 180.0f;
constexpr float min_longtitude = -180.0f;
constexpr float max_latitude = 90.0f;
constexpr float min_latitude = -90.0f;
constexpr int max_degree = 360;
constexpr int min_degree = 0;
constexpr int max_mag_degree_accuracy = 180;

constexpr float max_dynamics_g = 4.0f;                  // maximum dynamics in g
constexpr float max_altitude_meters = 50000.0f;         // maximum altitude in meters
constexpr float min_altitude_meters = -50000.0f;        // minimum altitude in meters
constexpr float max_velocity_mps = 500.0f;              // maximum velocity in meters per second
constexpr float min_velocity_mps = -500.0f;             // minimum velocity in meters per second
constexpr float velocity_accuracy_threshold_mps = 0.05f; // velocity accuracy in meters per second
constexpr float heading_accuracy_degrees = 0.3f;        // heading accuracy in degrees

constexpr float horizontal_accuracy_gps_glonass_m = 2.5f; // horizontal position accuracy for gps & glonass in meters
constexpr float horizontal_accuracy_galileo_m = 8.0f;     // horizontal position accuracy for galileo in meters (to be confirmed)

constexpr int max_navigation_update_rate_hz_gps = 10;     // max navigation update rate for gps in hz
constexpr int max_navigation_update_rate_hz_other = 18;   // max navigation update rate for glonass and galileo in hz

constexpr int cold_start_ttff_seconds = 26;  // time-to-first-fix for cold start in seconds
constexpr int hot_start_ttff_seconds = 1;    // time-to-first-fix for hot start in seconds
constexpr int aided_start_ttff_seconds = 2;  // time-to-first-fix for aided starts in seconds

constexpr int sensitivity_track_nav_dbm = -165;       // sensitivity for tracking & navigation in dbm
constexpr int sensitivity_reacquisition_dbm = -158;   // sensitivity for reacquisition in dbm
constexpr int sensitivity_cold_hot_start_dbm = -146;  // sensitivity for cold and hot starts in dbm

constexpr int measurement_period_millis_1_sec = 1000;  // measurement period in milliseconds
constexpr int measurement_period_millis_100_ms = 100;  // measurement period in milliseconds
constexpr int measurement_period_millis_25_ms = 25;    // measurement period in milliseconds

constexpr float longtitude_scale = 1e-07f;
constexpr float lattitude_scale = 1e-07f;
constexpr float motion_heading_scale = 1e-05f;
constexpr float motion_heading_accuracy_scale = 1e-05f;
constexpr float vehicle_heading_scale = 1e-05f;
constexpr float magnetic_declination_scale = 1e-02f;
constexpr float magnetic_declination_accuracy_scale = 1e-02f;

constexpr int default_navigation_rate = 1;
constexpr int default_time_ref = 0;

/** error handling */
constexpr int invalid_year_flag = 0xbeef;
static constexpr int invalid_sync_flag = 255;


#endif // GTU7_CONSTANTS_HPP
