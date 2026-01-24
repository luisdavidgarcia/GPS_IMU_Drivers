/*
 * gps.cpp
 *
 *  Created on: Jan 16, 2026
 */

#include "gtu7/gtu7.hpp"

/**
 * @brief   Constructor for the GPS class.
 *
 * Initializes the GPS module communication, sets message send rates, and measurement frequencies.
 */
GTU7::GTU7(UART_HandleTypeDef *huart)
        : pvtData_ {}, m_huart_(huart)
{
    // Enable UART here for GPS
    // Set baud_rate, message rate, measurement frequency
    ubx_setup();
}

/**
 * @brief   Configure the GPS module to use UBX protocol exclusively.
 *
 * Sends a UBX configuration message to the GPS module to use UBX protocol only.
 */
void GTU7::ubx_setup()
{
    // Send UART configuration
    UBX::Message setupMessage {
            .header {
                .msg_class { UBX::Msg_class::cfg },
                .msg_id { UBX::cfg_prt },
                .payload_length { 20 },
            },
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
    setupMessage.checksum = UBX::compute_checksum(setupMessage);
    if (!write_ubx_message(setupMessage)) {
        return;
    }
    // Get the ACK
    UBX::Message ack_message{};
    while (!read_ubx_message(ack_message, 10)) {
        HAL_Delay(1000);
    }

    // Set Measurement Frequency
    UBX::Message
    setupMeasurementFrequencyMessage {
        .header {
            .msg_class { UBX::Msg_class::cfg },
            .msg_id { UBX::cfg_rate },
            .payload_length { 6 },
        },
        .payload = {
            0xE8, 0x03,  // measRate
            0x01, 0x00,  // navRate
            0x00, 0x00   // timeRef
        }
    };
    UBX::compute_checksum(setupMeasurementFrequencyMessage);
    if (!write_ubx_message(setupMeasurementFrequencyMessage)) {
        return;
    }

    // Get the ACK
    ack_message = {};
    while (!read_ubx_message(ack_message, 10)) {
        HAL_Delay(1000);
    }
}

/**
 * @brief   Write a UBX message to the GPS module.
 *
 * @return  true if the message was successfully written, false otherwise.
 */
bool GTU7::write_ubx_message(UBX::Message &message) const
{
    std::vector < uint8_t > tempBuf;
    tempBuf.reserve(8 + message.header.payload_length);
    tempBuf.push_back(UBX::sync_char_1);
    tempBuf.push_back(UBX::sync_char_2);
    tempBuf.push_back(static_cast<uint8_t>(message.header.msg_class));
    tempBuf.push_back(message.header.msg_id);
    tempBuf.push_back(message.header.payload_length & byte_mask);
    tempBuf.push_back(message.header.payload_length >> byte_shift_amount);
    for (int i = 0; i < message.header.payload_length; i++) {
        tempBuf.push_back(message.payload.at(i));
    }
    tempBuf.push_back(message.checksum.a);
    tempBuf.push_back(message.checksum.b);

    HAL_StatusTypeDef status = HAL_UART_Transmit(m_huart_, tempBuf.data(),
            tempBuf.size(), default_timeout);
    if (status != HAL_OK) {
        return false;
    }

    return true;
}

/**
 * @brief   Read a UBX message from the GPS module.
 *
 * @return  The read UBX message.
 */
bool GTU7::read_ubx_message(UBX::Message& message, uint16_t messageSize)
{

    // Read the Header (6 Bytes) (sync 1, sync 2, msgClass, msgID, length (2 bytes))
    // Read the payload based on length
    // Read the checksum
    // Verify the checksum
    std::vector<uint8_t> buffer(messageSize);
    auto status = HAL_UART_Receive(m_huart_, buffer.data(), messageSize, default_timeout);
    if (status != HAL_OK) {
        return false;
    }

    if (buffer[0] == UBX::sync_char_1 && buffer[1] == UBX::sync_char_2) {
        message.header.msg_class = static_cast<UBX::Msg_class>(buffer[2]);
        message.header.msg_id = buffer[3];

        message.header.payload_length = (buffer[5] << byte_shift_amount | buffer[4]);
        if (message.header.payload_length + 8 != messageSize) {
            return false;
        }

        uint8_t payload_offset = 6;
        message.payload.resize(message.header.payload_length);
        std::copy(buffer.begin() + payload_offset,
                buffer.begin() + payload_offset + message.header.payload_length,
                message.payload.begin());

        size_t checksum_index = 6 + message.header.payload_length;
        message.checksum = {
            buffer[checksum_index],
            buffer[checksum_index + 1]
        };
        if (compute_checksum(message) == message.checksum) {
            return true;
        }
    }

    return false;
}

// TODO: Replaced this with the UBLOX-6 standard
/**
 * @brief   Retrieve Position-Velocity-Time (PVT) data from the GPS module.
 *
 * @return  The PVTData structure containing GPS-related information.
 */
PVT_data GTU7::get_pvt()
{
    // First send a request
    UBX::Message requestPosllh {
        .header {
            .msg_class { UBX::Msg_class::nav },
            .msg_id { UBX::nav_posllh },
            .payload_length { 0 },
        },
        .payload = {}
    };
    UBX::compute_checksum(requestPosllh);
    if (!write_ubx_message(requestPosllh)) {
        return pvtData_;
    }

    // Now read the data
    // TODO: Add 8 to default message size then add payload
    UBX::Message message{};
    if (read_ubx_message(message, 8 + 28)) {

//        uint32_t iTOW = u4_to_int(std::span<const uint8_t, 4>(&message.payload[0], 4));
        // Extract longitude and latitude in degrees
        pvtData_.longitude = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[4], 4)))
                * longtitude_scale;
        pvtData_.latitude = static_cast<float>(i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[8], 4)))
                * lattitude_scale;
        pvtData_.height = i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[12], 4));
        pvtData_.heightMSL = i4_to_int(
                std::span<const uint8_t, 4>(&message.payload[16], 4));

        return pvtData_;
    }

    return pvtData_;
}

/**
 * @brief   Convert a little-endian byte array to a signed 16-bit integer.
 *
 * @param   little_endian_bytes The input byte array.
 * @return  The converted signed 16-bit integer.
 */
int16_t GTU7::i2_to_int(std::span<const uint8_t, 2> bytes)
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
uint16_t GTU7::u2_to_int(std::span<const uint8_t, 2> bytes)
{
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
int32_t GTU7::i4_to_int(std::span<const uint8_t, 4> bytes)
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
uint32_t GTU7::u4_to_int(std::span<const uint8_t, 4> bytes)
{
    auto byte0 = static_cast<uint32_t>(bytes[0]);
    auto byte1 = static_cast<uint32_t>(bytes[1]) << byte_shift_amount;
    auto byte2 = static_cast<uint32_t>(bytes[2]) << half_word_shift_amount;
    auto byte3 = static_cast<uint32_t>(bytes[3]) << three_byte_shift_amount;

    return (byte3 | byte2 | byte1 | byte0);
}
