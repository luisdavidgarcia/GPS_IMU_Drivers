/*
 * gps.cpp
 *
 *  Created on: Jan 16, 2026
 */

#include "gps/gps.hpp"

/**
 * @brief   Constructor for the GPS class.
 *
 * Initializes the GPS module communication, sets message send rates, and measurement frequencies.
 */
GPS::GPS(int16_t currentYear, UART_HandleTypeDef *huart)
        : currentYear_(currentYear), pvtData_ {}, m_huart_(huart)
{

    if (currentYear == default_year) {
        printf("Error: Current year is not set.\n");
        exit(-1);
    }

    // Enable UART here for GPS
    // Set baud_rate, message rate, measurement frequency
    ubx_setup();
}

/**
 * @brief   Configure the GPS module to use UBX protocol exclusively.
 *
 * Sends a UBX configuration message to the GPS module to use UBX protocol only.
 */
void GPS::ubx_setup()
{
    // Send UART configuration
    UBX::UBX_message setupMessage {
            .msg_class { UBX::Msg_class::cfg },
            .msg_id { UBX::cfg_prt },
            .payload_length { 20 },
            .payload = {
                    0x01,					    // PORT
                    0x00,						// Reserved0
                    0x00, 0x00,					// txReady
                    0xD0, 0x08, 0x00, 0x00,		// mode
                    0x00, 0xC2, 0x01, 0x00,		// baudRate
                    0x01, 0x00,					// inProtoMask
                    0x01, 0x00,					// outProtoMask
                    0x00, 0x00,					// reserved4
                    0x00, 0x00 					// reserved5
                    },
    };
    ubx_.compute_checksum(setupMessage);
    bool result = write_ubx_message(setupMessage);
    if (!result) {
        exit(-1);
    }

    // Set Measurement Frequency
    UBX::UBX_message
    setupMeasurementFrequencyMessage { .msg_class { UBX::Msg_class::cfg }, .msg_id
            { UBX::cfg_rate }, .payload_length { 6 }, .payload = { 0xE8, 0x03, // measRate
            0x01, 0x00,  // navRate
            0x00, 0x00   // timeRef
            } };
    ubx_.compute_checksum(setupMeasurementFrequencyMessage);
    result = write_ubx_message(setupMeasurementFrequencyMessage);
    if (!result) {
        exit(-1);
    }
}

/**
 * @brief   Write a UBX message to the GPS module.
 *
 * @return  true if the message was successfully written, false otherwise.
 */
bool GPS::write_ubx_message(UBX::UBX_message &message) const
{
    std::vector < uint8_t > tempBuf;
    tempBuf.push_back(message.sync1);
    tempBuf.push_back(message.sync2);
    tempBuf.push_back(static_cast<uint8_t>(message.msg_class));
    tempBuf.push_back(message.msg_id);
    tempBuf.push_back(message.payload_length & byte_mask);
    tempBuf.push_back(message.payload_length >> byte_shift_amount);
    for (int i = 0; i < message.payload_length; i++) {
        tempBuf.push_back(message.payload.at(i));
    }
    tempBuf.push_back(message.checksum_a);
    tempBuf.push_back(message.checksum_b);

    HAL_StatusTypeDef status = HAL_UART_Transmit(m_huart_, tempBuf.data(),
            tempBuf.size(), default_timeout);
    if (status == HAL_ERROR) {
        perror("Failed to write to I2C device");
        return false;
    }

    return true;
}

/**
 * @brief   Read a UBX message from the GPS module.
 *
 * @return  The read UBX message.
 */
UBX::UBX_message GPS::read_ubx_message(uint16_t messageSize)
{
    std::vector < uint8_t > message(messageSize);
    UBX::UBX_message badMsg = { invalid_sync_flag };
    HAL_StatusTypeDef status = HAL_UART_Receive(m_huart_, message.data(),
            messageSize, default_timeout);
    if (status == HAL_ERROR) {
        return badMsg;
    }

    if (message[0] == UBX::sync_char_1 && message[1] == UBX::sync_char_2) {
        UBX::UBX_message ubxMsg {};
        ubxMsg.sync1 = message[0];
        ubxMsg.sync2 = message[1];
        ubxMsg.msg_class = static_cast<UBX::Msg_class>(message[2]);
        ubxMsg.msg_id = message[3];
        ubxMsg.payload_length = (message[5] << byte_shift_amount | message[4]);
        if (ubxMsg.payload_length > messageSize) {
            return badMsg;
        }
        uint8_t payload_offset = 6;
        std::copy(message.begin() + payload_offset,
                message.begin() + payload_offset + ubxMsg.payload_length,
                ubxMsg.payload.begin());
        ubxMsg.checksum_a = message[message.size() - 2];
        ubxMsg.checksum_b = message[message.size() - 1];

        return ubxMsg;
    }

    return badMsg;
}

/**
 * @brief   Retrieve Position-Velocity-Time (PVT) data from the GPS module.
 *
 * @return  The PVTData structure containing GPS-related information.
 */
PVT_data GPS::get_pvt()
{
    uint16_t messageSize { 94 };
    UBX::UBX_message message = read_ubx_message(messageSize);

    if (message.sync1 != invalid_sync1_flag) {
        pvtData_.year = u2_to_int(
                std::span<const uint8_t, 2>(&message.payload[4], 2));
        if (pvtData_.year != currentYear_) {
            pvtData_.year = invalid_year_flag;
            return this->pvtData_;
        }
        pvtData_.month = message.payload[6];
        pvtData_.day = message.payload[7];
        pvtData_.hour = message.payload[8];
        pvtData_.min = message.payload[9];
        pvtData_.sec = message.payload[10];
        uint8_t valid_flag = message.payload[11];

        // Extract and clarify flags
        pvtData_.validDateFlag =
                (valid_flag & valid_date_flag) == valid_date_flag ? 1 : 0;
        pvtData_.validTimeFlag =
                (valid_flag & valid_time_flag) == valid_time_flag ? 1 : 0;
        pvtData_.fullyResolved =
                (valid_flag & fully_resolved_flag) == fully_resolved_flag ?
                        1 : 0;
        pvtData_.validMagFlag =
                (valid_flag & valid_mag_flag) == valid_mag_flag ? 1 : 0;

        // Extract GNSS fix and related data
        pvtData_.gnssFix = message.payload[20];
        memcpy(&pvtData_.fixStatusFlags, &message.payload[21], 2);
        pvtData_.numberOfSatellites = message.payload[23];

        // Extract longitude and latitude in degrees
        pvtData_.longitude = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[24], 4)))
                * longtitude_scale;
        pvtData_.latitude = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[28], 4)))
                * lattitude_scale;
        pvtData_.height = i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[32], 4));
        pvtData_.heightMSL = i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[36], 4));
        // Extract horizontal and vertical accuracy in millimeters
        pvtData_.horizontalAccuracy = u4_to_int(
                std::span<const uint8_t, 4>(&message.payload[40], 4));
        pvtData_.verticalAccuracy = u4_to_int(
                std::span<const uint8_t, 4>(&message.payload[44], 4));
        // Extract North East Down velocity in mm/s
        pvtData_.velocityNorth = i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[48], 4));
        pvtData_.velocityEast = i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[52], 4));
        pvtData_.velocityDown = i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[56], 4));
        // Extract ground speed in mm/s and motion heading in degrees
        pvtData_.groundSpeed = i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[60], 4));
        pvtData_.motionHeading = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[64], 4)))
                * motion_heading_scale;
        // Extract speed accuracy in mm/s and heading accuracy in degrees
        pvtData_.speedAccuracy = u4_to_int(
                std::span<const uint8_t, 4>(&message.payload[68], 4));
        pvtData_.motionHeadingAccuracy = static_cast<float>(u4_to_int(
                std::span<const uint8_t, 4>(&message.payload[72], 4)))
                * motion_heading_accuracy_scale;
        // Extract vehicle heading in degrees
        pvtData_.vehicalHeading = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[84], 4)))
                * vehicle_heading_scale;
        // Extract magnetic declination and accuracy in degrees
        pvtData_.magneticDeclination = static_cast<float>(i2_to_int(
                std::span<const uint8_t, 2>(&message.payload[88], 4)))
                * magnetic_declination_scale;
        pvtData_.magnetDeclinationAccuracy = static_cast<float>(u2_to_int(
                std::span<const uint8_t, 2>(&message.payload[90], 4)))
                * magnetic_declination_accuracy_scale;

        return this->pvtData_;
    }

    pvtData_.year = invalid_year_flag;
    return this->pvtData_;
}

/**
 * @brief   Convert a little-endian byte array to a signed 16-bit integer.
 *
 * @param   little_endian_bytes The input byte array.
 * @return  The converted signed 16-bit integer.
 */
int16_t GPS::i2_to_int(std::span<const uint8_t, 2> bytes)
{
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
uint16_t GPS::u2_to_int(std::span<const uint8_t, 2> bytes)
{
    auto byte0 = static_cast<uint16_t>(bytes[0]);
    auto byte1 = static_cast<uint16_t>(bytes[1]) << byte_shift_amount;

    return byte1 | byte0;
}

double GPS::bytes_to_double(const uint8_t *little_endian_bytes)
{
    // Assuming little_endian_bytes is a representation of a double
    // If it's not, you'll need to adjust the conversion logic accordingly
    double result = 0;
    memcpy(&result, little_endian_bytes, sizeof(result));
    return result * 1e-07;
}

/**
 * @brief   Convert a little-endian byte array to a signed 32-bit integer.
 *
 * @param   little_endian_bytes The input byte array.
 * @return  The converted signed 32-bit integer.
 */
int32_t GPS::i4_to_int(std::span<const uint8_t, 4> bytes)
{
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
uint32_t GPS::u4_to_int(std::span<const uint8_t, 4> bytes)
{
    auto byte0 = static_cast<uint32_t>(bytes[0]);
    auto byte1 = static_cast<uint32_t>(bytes[1]) << byte_shift_amount;
    auto byte2 = static_cast<uint32_t>(bytes[2]) << half_word_shift_amount;
    auto byte3 = static_cast<uint32_t>(bytes[3]) << three_byte_shift_amount;

    return (byte3 | byte2 | byte1 | byte0);
}
