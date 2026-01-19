/*
 * gps.hpp
 *
 *  Created on: Jan 16, 2026
 */

/*
 * GPS.h - Header file for GPS module integration
 *
 * This header file defines the interface for interacting with a GPS module
 * over the I2C communication protocol. It provides functions and data structures
 * to retrieve GPS-related information, including location, time, velocity, and more.
 * The code in this header is designed to work with a specific GPS module and may
 * require configuration and adaptation for different hardware or use cases.
 *
 *
 * Dependencies:
 * - ubx/ubx_msg.hpp: Defines UBX message structures for communication with the GPS module.
 * - fcntl.h, unistd.h, i2c/smbus.h, linux/i2c-dev.h, linux/i2c.h: Required for I2C communication.
 * - cstdint, sys/ioctl.h, string, chrono, thread, vector, cstring: Standard C++ libraries.
 *
 * Constants and Definitions:
 * - Various constants for I2C addresses, register addresses, timeouts, and default values.
 *
 * Data Structures:
 * - PVTData: A structure to hold GPS-related data, including time, coordinates, accuracy,
 *   velocity, and more.
 *
 * Class:
 * - Gps: A class that encapsulates GPS functionality, including I2C communication,
 *   message handling, and data retrieval.
 *
 * Usage:
 * - Include this header file in your C++ project to interact with a GPS module.
 * - Instantiate the Gps class to communicate with the GPS module and retrieve data.
 *
 * Note: This code is designed for the GT-U7 GPS, but may be adapted to other modules.
 * 			It is also UART based and thus is a poll based mechanism of transmit
 * 			and receive for the data.
 */

#ifndef GPS_HPP
#define GPS_HPP

extern "C"
{
#include "stm32l4xx_hal.h"
}

#include "ubx/ubx.hpp"
#include "mpu9250_imu/mpu9250_register_map.hpp"
#include "gps/gps_constants.hpp"

#include <fcntl.h>
#include <span>
#include <unistd.h>
#include <cstdint>
#include <string>
#include <vector>
#include <cstring>
#include <cstdio>

struct alignas(128) PVT_data
{
    // Time Information
    uint16_t year;                        // Year (UTC)
    uint8_t month;                        // Month (UTC)
    uint8_t day;                          // Day of the month (UTC)
    uint8_t hour;                         // Hour of the day (UTC)
    uint8_t min;                          // Minute of the hour (UTC)
    uint8_t sec;                          // Second of the minute (UTC)

    // Validity Flags
    uint8_t validTimeFlag;                // Validity flags for time
    uint8_t validDateFlag;                // Validity flags for time
    uint8_t fullyResolved;                // Validity flags for time
    uint8_t validMagFlag;                 // Validity flags for time

    // GNSS
    uint8_t gnssFix;
    uint8_t fixStatusFlags;
    uint8_t numberOfSatellites;

    // Coordinates
    float longitude;
    float latitude;                       // Latitude (degrees * 1e7)
    int32_t height;                      // Height above ellipsoid (millimeters)
    int32_t heightMSL;              // Height above mean sea level (millimeters)

    // Accuracy Information
    uint32_t horizontalAccuracy;   // Horizontal accuracy estimate (millimeters)
    uint32_t verticalAccuracy;       // Vertical accuracy estimate (millimeters)

    // Velocity and Heading
    int32_t velocityNorth; // Velocity in the north direction (millimeters/second)
    int32_t velocityEast; // Velocity in the east direction (millimeters/second)
    int32_t velocityDown; // Velocity in the down direction (millimeters/second)
    int32_t groundSpeed;                  // Ground speed (millimeters/second)
    float vehicalHeading;
    float motionHeading;                  // Heading of motion (degrees * 1e5)
    uint32_t speedAccuracy;      // Speed accuracy estimate (millimeters/second)
    float motionHeadingAccuracy;

    // Magnetic Declination
    float magneticDeclination;           // Magnetic declination (degrees * 1e2)
    float magnetDeclinationAccuracy;     // Declination accuracy (degrees * 1e2)
};

class GPS
{
public:
    explicit GPS(int16_t currentYear, UART_HandleTypeDef *huart);
    PVT_data get_pvt();

private:
    int16_t currentYear_;
    PVT_data pvtData_;
    UBX ubx_;
    UART_HandleTypeDef *m_huart_;

    void ubx_setup();
    bool write_ubx_message(UBX::UBX_message &message) const;
    UBX::UBX_message read_ubx_message(uint16_t messageSize);

    static double bytes_to_double(const uint8_t *little_endian_bytes);
    static int16_t i2_to_int(std::span<const uint8_t, 2> bytes);
    static uint16_t u2_to_int(std::span<const uint8_t, 2> bytes);
    static int32_t i4_to_int(std::span<const uint8_t, 4> bytes);
    static uint32_t u4_to_int(std::span<const uint8_t, 4> bytes);

};

#endif // GPS_HPP

