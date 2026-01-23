/*
 * gtu7.hpp
 *
 *  Created on: Jan 16, 2026
 */

#ifndef GTU7_HPP
#define GTU7_HPP

extern "C"
{
#include "stm32l4xx_hal.h"
}

#include "ubx/ubx.hpp"
#include <mpu9250/mpu9250_register_map.hpp>
#include <gtu7/gtu7_constants.hpp>

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

class GTU7
{
public:
    explicit GTU7(int16_t currentYear, UART_HandleTypeDef *huart);
    PVT_data get_pvt();

private:
    int16_t currentYear_;
    PVT_data pvtData_;
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

#endif // GTU7_HPP

