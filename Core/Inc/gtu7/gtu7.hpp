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

struct alignas(128) Polssh
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

static constexpr uint8_t header_bytes = 6;
static constexpr uint8_t checksum_bytes = 2;

class GTU7
{
public:
    explicit GTU7(UART_HandleTypeDef *huart);
    bool get_posllh(Polssh& data, uint8_t* buffer, size_t buffer_size);

private:
    UART_HandleTypeDef *m_huart_;

    void ubx_setup();
    void set_configuration();
    void set_measurement_frequency();
    void set_posllh();
    void disable_nmea();

    bool write_ubx_message(const UBX::Message& message) const;
    bool read_ubx_message(UBX::Message& message);
    bool wait_for_ack(uint8_t msg_id, uint32_t timeout_ms);

    /**
     * @brief   Convert a little-endian byte array to a signed 16-bit integer.
     *
     * @param   little_endian_bytes The input byte array.
     * @return  The converted signed 16-bit integer.
     */
    static constexpr inline int16_t i2_to_int(std::span<const uint8_t, 2> bytes) noexcept {
        auto byte0 = static_cast<uint16_t>(bytes[0]);
        auto byte1 = static_cast<uint16_t>(bytes[1]) << byte_shift_amount;

        return static_cast<int16_t>(byte1 | byte0);
    }

    /**
     * @brief   Convert a little-endian byte array to an unsigned 16-bit integer.
     *
     * @param   little_endian_bytes The input byte array.
     * @return  The converted unsigned 16-bit integer.
     */
    static constexpr inline uint16_t u2_to_int(std::span<const uint8_t, 2> bytes) noexcept {
        auto byte0 = static_cast<uint16_t>(bytes[0]);
        auto byte1 = static_cast<uint16_t>(bytes[1]) << byte_shift_amount;

        return byte1 | byte0;
    }

    /**
     * @brief   Convert a little-endian byte array to a signed 32-bit integer.
     *
     * @param   little_endian_bytes The input byte array.
     * @return  The converted signed 32-bit integer.
     */
    static constexpr inline int32_t i4_to_int(std::span<const uint8_t, 4> bytes) noexcept {
        auto byte0 = static_cast<uint32_t>(bytes[0]);
        auto byte1 = static_cast<uint32_t>(bytes[1]) << byte_shift_amount;
        auto byte2 = static_cast<uint32_t>(bytes[2]) << half_word_shift_amount;
        auto byte3 = static_cast<uint32_t>(bytes[3]) << three_byte_shift_amount;

        return static_cast<int32_t>(byte3 | byte2 | byte1 | byte0);
    }

    /**
     * @brief   Convert a little-endian byte array to an unsigned 32-bit integer.
     *
     * @param   little_endian_bytes The input byte array.
     * @return  The converted unsigned 32-bit integer.
     */
    static constexpr inline uint32_t u4_to_int(std::span<const uint8_t, 4> bytes) noexcept {
        auto byte0 = static_cast<uint32_t>(bytes[0]);
        auto byte1 = static_cast<uint32_t>(bytes[1]) << byte_shift_amount;
        auto byte2 = static_cast<uint32_t>(bytes[2]) << half_word_shift_amount;
        auto byte3 = static_cast<uint32_t>(bytes[3]) << three_byte_shift_amount;

        return (byte3 | byte2 | byte1 | byte0);
    }

};

#endif // GTU7_HPP

