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

    if (currentYear == DEFAULT_YEAR) {
        printf("Error: Current year is not set.\n");
        exit(-1);
    }

    // Enable UART here for GPS
    // Set baud_rate, message rate, measurement frequency
    ubxSetup();
}

/**
 * @brief   Configure the GPS module to use UBX protocol exclusively.
 *
 * Sends a UBX configuration message to the GPS module to use UBX protocol only.
 */
void GPS::ubxSetup()
{
    // Send UART configuration
    UBX::UbxMessage
    setupMessage { .msgClass { UBX::MsgClass::cfg }, .msgID { UBX::cfg_prt },
            .payloadLength { 20 }, .payload = { 0x01,					// PORT
                    0x00,						// Reserved0
                    0x00, 0x00,					// txReady
                    0xD0, 0x08, 0x00, 0x00,		// mode
                    0x00, 0xC2, 0x01, 0x00,		// baudRate
                    0x01, 0x00,					// inProtoMask
                    0x01, 0x00,					// outProtoMask
                    0x00, 0x00,					// reserved4
                    0x00, 0x00 					// reserved5
                    } };
    ubx_.ComputeChecksum(setupMessage);
    bool result = writeUbxMessage(setupMessage);
    if (!result) {
        exit(-1);
    }

    // Set Measurement Frequency
    UBX::UbxMessage
    setupMeasurementFrequencyMessage { .msgClass { UBX::MsgClass::cfg }, .msgID
            { UBX::cfg_rate }, .payloadLength { 6 }, .payload = { 0xE8, 0x03, // measRate
            0x01, 0x00,  // navRate
            0x00, 0x00   // timeRef
            } };
    ubx_.ComputeChecksum(setupMeasurementFrequencyMessage);
    result = writeUbxMessage(setupMeasurementFrequencyMessage);
    if (!result) {
        exit(-1);
    }
}

/**
 * @brief   Write a UBX message to the GPS module.
 *
 * @return  true if the message was successfully written, false otherwise.
 */
bool GPS::writeUbxMessage(UBX::UbxMessage &message) const
{
    std::vector < uint8_t > tempBuf;
    tempBuf.push_back(message.sync1);
    tempBuf.push_back(message.sync2);
    tempBuf.push_back(static_cast<uint8_t>(message.msgClass));
    tempBuf.push_back(message.msgID);
    tempBuf.push_back(message.payloadLength & BYTE_MASK);
    tempBuf.push_back(message.payloadLength >> BYTE_SHIFT_AMOUNT);
    for (int i = 0; i < message.payloadLength; i++) {
        tempBuf.push_back(message.payload.at(i));
    }
    tempBuf.push_back(message.checksumA);
    tempBuf.push_back(message.checksumB);

    HAL_StatusTypeDef status = HAL_UART_Transmit(m_huart_, tempBuf.data(),
            tempBuf.size(), DEFAULT_TIMEOUT);
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
UBX::UbxMessage GPS::readUbxMessage(uint16_t messageSize)
{
    std::vector < uint8_t > message(messageSize);
    UBX::UbxMessage badMsg = { INVALID_SYNC_FLAG };
    HAL_StatusTypeDef status = HAL_UART_Receive(m_huart_, message.data(),
            messageSize, DEFAULT_TIMEOUT);
    if (status == HAL_ERROR) {
        return badMsg;
    }

    if (message[0] == UBX::sync_char_1 && message[1] == UBX::sync_char_2) {
        UBX::UbxMessage ubxMsg {};
        ubxMsg.sync1 = message[0];
        ubxMsg.sync2 = message[1];
        ubxMsg.msgClass = static_cast<UBX::MsgClass>(message[2]);
        ubxMsg.msgID = message[3];
        ubxMsg.payloadLength = (message[5] << BYTE_SHIFT_AMOUNT | message[4]);
        if (ubxMsg.payloadLength > messageSize) {
            return badMsg;
        }
        uint8_t payload_offset = 6;
        std::copy(message.begin() + payload_offset,
                message.begin() + payload_offset + ubxMsg.payloadLength,
                ubxMsg.payload.begin());
        ubxMsg.checksumA = message[message.size() - 2];
        ubxMsg.checksumB = message[message.size() - 1];

        return ubxMsg;
    }

    return badMsg;
}

/**
 * @brief   Retrieve Position-Velocity-Time (PVT) data from the GPS module.
 *
 * @return  The PVTData structure containing GPS-related information.
 */
PVTData GPS::GetPvt()
{
    uint16_t messageSize { 94 };
    UBX::UbxMessage message = readUbxMessage(messageSize);

    if (message.sync1 != INVALID_SYNC1_FLAG) {
        pvtData_.year = u2_to_int(
                std::span<const uint8_t, 2>(&message.payload[4], 2));
        if (pvtData_.year != currentYear_) {
            pvtData_.year = INVALID_YEAR_FLAG;
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
                (valid_flag & VALID_DATE_FLAG) == VALID_DATE_FLAG ? 1 : 0;
        pvtData_.validTimeFlag =
                (valid_flag & VALID_TIME_FLAG) == VALID_TIME_FLAG ? 1 : 0;
        pvtData_.fullyResolved =
                (valid_flag & FULLY_RESOLVED_FLAG) == FULLY_RESOLVED_FLAG ?
                        1 : 0;
        pvtData_.validMagFlag =
                (valid_flag & VALID_MAG_FLAG) == VALID_MAG_FLAG ? 1 : 0;

        // Extract GNSS fix and related data
        pvtData_.gnssFix = message.payload[20];
        memcpy(&pvtData_.fixStatusFlags, &message.payload[21], 2);
        pvtData_.numberOfSatellites = message.payload[23];

        // Extract longitude and latitude in degrees
        pvtData_.longitude = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[24], 4)))
                * LONGTITUDE_SCALE;
        pvtData_.latitude = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[28], 4)))
                * LATTITUDE_SCALE;
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
                * MOTION_HEADING_SCALE;
        // Extract speed accuracy in mm/s and heading accuracy in degrees
        pvtData_.speedAccuracy = u4_to_int(
                std::span<const uint8_t, 4>(&message.payload[68], 4));
        pvtData_.motionHeadingAccuracy = static_cast<float>(u4_to_int(
                std::span<const uint8_t, 4>(&message.payload[72], 4)))
                * MOTION_HEADING_ACCURACY_SCALE;
        // Extract vehicle heading in degrees
        pvtData_.vehicalHeading = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[84], 4)))
                * VEHICLE_HEADING_SCALE;
        // Extract magnetic declination and accuracy in degrees
        pvtData_.magneticDeclination = static_cast<float>(i2_to_int(
                std::span<const uint8_t, 2>(&message.payload[88], 4)))
                * MAGNETIC_DECLINATION_SCALE;
        pvtData_.magnetDeclinationAccuracy = static_cast<float>(u2_to_int(
                std::span<const uint8_t, 2>(&message.payload[90], 4)))
                * MAGNETIC_DECLINATION_ACCURACY_SCALE;

        return this->pvtData_;
    }

    pvtData_.year = INVALID_YEAR_FLAG;
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
    auto byte1 = static_cast<uint16_t>(bytes[1]) << BYTE_SHIFT_AMOUNT;

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
    auto byte1 = static_cast<uint16_t>(bytes[1]) << BYTE_SHIFT_AMOUNT;

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
    auto byte1 = static_cast<uint32_t>(bytes[1]) << BYTE_SHIFT_AMOUNT;
    auto byte2 = static_cast<uint32_t>(bytes[2]) << HALF_WORD_SHIFT_AMOUNT;
    auto byte3 = static_cast<uint32_t>(bytes[3]) << THREE_BYTE_SHIFT_AMOUNT;

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
    auto byte1 = static_cast<uint32_t>(bytes[1]) << BYTE_SHIFT_AMOUNT;
    auto byte2 = static_cast<uint32_t>(bytes[2]) << HALF_WORD_SHIFT_AMOUNT;
    auto byte3 = static_cast<uint32_t>(bytes[3]) << THREE_BYTE_SHIFT_AMOUNT;

    return (byte3 | byte2 | byte1 | byte0);
}
