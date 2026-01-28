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
        : m_huart_(huart), pvtData_ {}
{
    // Enable UART here for GPS
    // Set baud_rate, message rate, measurement frequency
    HAL_Delay(1000);

    // Configure GPS
    ubx_setup();
}

void GTU7::set_configuration() {
    // Send UART configuration
    UBX::Message setupMessage {
            .header {
                .msg_class { UBX::Msg_class::cfg },
                .msg_id { UBX::cfg_prt },
                .payload_length { 20 },
            },
            .payload = {
                    0x01,                       // PORT
                    0x00,                       // Reserved0
                    0x00, 0x00,                 // txReady
                    0xD0, 0x08, 0x00, 0x00,     // mode
                    0x80, 0x25, 0x00, 0x00,     // baudRate
                    0x01, 0x00,                 // inProtoMask
                    0x01, 0x00,                 // outProtoMask (change to 0x01 after debugging to remove NMEA)
                    0x00, 0x00,                 // reserved4
                    0x00, 0x00                  // reserved5
                    },
    };
    setupMessage.checksum = UBX::compute_checksum(setupMessage);
    if (!write_ubx_message(setupMessage)) {
        return;
    }
    HAL_Delay(200);
    __HAL_UART_FLUSH_DRREGISTER(m_huart_);

    // Get the ACK
//    UBX::Message ack_message = {};
//    while (!read_ubx_message(ack_message)) {
//        HAL_Delay(1000);
//    }
}

void GTU7::set_measurement_frequency() {
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
    setupMeasurementFrequencyMessage.checksum =
            UBX::compute_checksum(setupMeasurementFrequencyMessage);
    if (!write_ubx_message(setupMeasurementFrequencyMessage)) {
        return;
    }
    HAL_Delay(100);

    // Get the ACK
//    UBX::Message ack_message = {};
//    while (!read_ubx_message(ack_message)) {
//        HAL_Delay(1000);
//    }
}

void GTU7::set_posllh() {
    // Enable POSLLH
    UBX::Message
        enablePosllh {
            .header {
                .msg_class { UBX::Msg_class::cfg },
                .msg_id { UBX::cfg_msg },
                .payload_length { 8 },
            },
            .payload = {
                    static_cast<uint8_t>(UBX::Msg_class::nav),
                    UBX::nav_posllh,
                    0x00,                                      // rateDDC
                    0x01,                                      // rateUART
                    0x00,                                      // rateUSB
                    0x00,                                      // rateSPI
                    0x00,                                      // reserved
                    0x00                                       // reserved
            }
    };
    enablePosllh.checksum = UBX::compute_checksum(enablePosllh);
    if (!write_ubx_message(enablePosllh)) {
        return;
    }
    HAL_Delay(100);
}


void GTU7::disable_nmea()
{
    // Disable NMEA (GGA, GLL, GSA, GSV, RMC, VTG, TXT )
    UBX::Message
        disableGGA {
            .header {
                .msg_class{ UBX::Msg_class::cfg},
                .msg_id { UBX::cfg_msg },
                .payload_length { 8 },
            },
            .payload = {
                0xF0,
                0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            }
    };
    disableGGA.checksum = UBX::compute_checksum(disableGGA);
    if (!write_ubx_message(disableGGA)) {
        return;
    }
    HAL_Delay(100);

    UBX::Message
        disableGLL {
            .header {
                .msg_class{ UBX::Msg_class::cfg},
                .msg_id { UBX::cfg_msg },
                .payload_length { 8 },
            },
            .payload = {
                0xF0,
                0x01,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            }
    };
    disableGLL.checksum = UBX::compute_checksum(disableGLL);
    if (!write_ubx_message(disableGLL)) {
        return;
    }
    HAL_Delay(100);

    UBX::Message
        disableGSA {
            .header {
                .msg_class{ UBX::Msg_class::cfg},
                .msg_id { UBX::cfg_msg },
                .payload_length { 8 },
            },
            .payload = {
                0xF0,
                0x02,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            }
    };
    disableGSA.checksum = UBX::compute_checksum(disableGSA);
    if (!write_ubx_message(disableGSA)) {
        return;
    }
    HAL_Delay(100);

    UBX::Message
        disableGSV {
            .header {
                .msg_class{ UBX::Msg_class::cfg},
                .msg_id { UBX::cfg_msg },
                .payload_length { 8 },
            },
            .payload = {
                0xF0,
                0x03,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            }
    };
    disableGSV.checksum = UBX::compute_checksum(disableGSV);
    if (!write_ubx_message(disableGSV)) {
        return;
    }
    HAL_Delay(1000);

    UBX::Message
        disableRMC {
            .header {
                .msg_class{ UBX::Msg_class::cfg},
                .msg_id { UBX::cfg_msg },
                .payload_length { 8 },
            },
            .payload = {
                0xF0,
                0x04,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            }
    };
    disableRMC.checksum = UBX::compute_checksum(disableRMC);
    if (!write_ubx_message(disableRMC)) {
        return;
    }
    HAL_Delay(100);

    UBX::Message
        disableVTG {
            .header {
                .msg_class{ UBX::Msg_class::cfg},
                .msg_id { UBX::cfg_msg },
                .payload_length { 8 },
            },
            .payload = {
                0xF0,
                0x05,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            }
    };
    disableVTG.checksum = UBX::compute_checksum(disableVTG);
    if (!write_ubx_message(disableVTG)) {
        return;
    }
    HAL_Delay(100);

    UBX::Message
        disableTXT {
            .header {
                .msg_class{ UBX::Msg_class::cfg},
                .msg_id { UBX::cfg_msg },
                .payload_length { 8 },
            },
            .payload = {
                0xF0,
                0x41,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            }
    };
    disableTXT.checksum = UBX::compute_checksum(disableTXT);
    if (!write_ubx_message(disableTXT)) {
        return;
    }
    HAL_Delay(100);
}

/**
 * @brief   Configure the GPS module to use UBX protocol exclusively.
 *
 * Sends a UBX configuration message to the GPS module to use UBX protocol only.
 */
void GTU7::ubx_setup()
{
    __HAL_UART_FLUSH_DRREGISTER(m_huart_);
    HAL_Delay(100);

    set_configuration();
    set_measurement_frequency();
    set_posllh();
    disable_nmea();

    // Give GPS time to stop sending NMEA
    HAL_Delay(500);
    __HAL_UART_FLUSH_DRREGISTER(m_huart_);
    HAL_Delay(100);
}

/**
 * @brief   Wait for ACK/NACK
 */
bool GTU7::wait_for_ack(uint8_t msg_id, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    UBX::Message ack_message{};

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (read_ubx_message(ack_message)) {
            if (ack_message.header.msg_class == UBX::Msg_class::ack &&
                    ack_message.header.msg_id == UBX::ack_ack &&
                    ack_message.payload[1] == msg_id) {
                return true;
            }

            if (ack_message.header.msg_class == UBX::Msg_class::ack &&
                    ack_message.header.msg_id == UBX::ack_nak &&
                    ack_message.payload[1] == msg_id) {
                return true;
            }
        }
        HAL_Delay(10);
    }

    return false;
}

/**
 * @brief   Write a UBX message to the GPS module.
 *
 * @return  true if the message was successfully written, false otherwise.
 */
bool GTU7::write_ubx_message(const UBX::Message& message) const
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
bool GTU7::read_ubx_message(UBX::Message& message)
{
    // Read the sync bytes (sync 1, sync 2)
    uint8_t sync_byte;
    auto status = HAL_UART_Receive(m_huart_, &sync_byte, 1, default_timeout);
    if (status != HAL_OK || sync_byte != UBX::sync_char_1) {
        return false;
    }

    status = HAL_UART_Receive(m_huart_, &sync_byte, 1, default_timeout);
    if (status != HAL_OK || sync_byte != UBX::sync_char_2) {
        return false;
    }

    // Read the Header (4 Bytes) (msgClass, msgID, length (2 bytes))
    uint8_t header[4];
    status = HAL_UART_Receive(m_huart_, header, header_bytes, default_timeout);
    if (status != HAL_OK) {
        return false;
    }
    message.header.msg_class = static_cast<UBX::Msg_class>(header[0]);
    message.header.msg_id = header[1];
    message.header.payload_length = (header[3] << byte_shift_amount | header[2]);


    // Read the payload based on length
    message.payload.resize(message.header.payload_length);
    status = HAL_UART_Receive(m_huart_, message.payload.data(), message.header.payload_length, default_timeout);
    if (status != HAL_OK) {
        return false;
    }

    // Read the checksum and verify
    uint8_t checksum[2];
    status = HAL_UART_Receive(m_huart_, checksum, checksum_bytes, default_timeout);
    if (status != HAL_OK) {
        return false;
    }
    message.checksum.a = checksum[0];
    message.checksum.b = checksum[1];
    if (message.checksum == compute_checksum(message)) {
        return true;
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
    // Now read the data
    // TODO: Add 8 to default message size then add payload
    std::vector<uint8_t> buff;
    while (UART4->ISR & USART_ISR_RXNE)  {
        uint8_t byte = static_cast<uint8_t>(UART4->RDR);
        buff.push_back(byte);
    }

    UBX::Message message{};
    if (read_ubx_message(message)) {
        if (message.header.msg_class != UBX::Msg_class::nav ||
            message.header.msg_id    != UBX::nav_posllh ||
            message.header.payload_length < 20) {
            return pvtData_;
        }

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
