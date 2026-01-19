#include "ubx/ubx.hpp"

/**
 * @brief   Calculate the checksum values (checksumA and checksumB) for a UBX message
 * @param 	Message to calculate the checksum for
 */
void UBX::compute_checksum(UBX_message &message)
{
    message.checksum_a = static_cast<uint8_t>(message.msg_class);
    message.checksum_b = message.checksum_a;

    message.checksum_a += message.msg_id;
    message.checksum_b += message.checksum_a;

    message.checksum_a += message.payload_length % (1 << 8);
    message.checksum_b += message.checksum_a;

    message.checksum_a += message.payload_length >> 8;
    message.checksum_b += message.checksum_a;

    for (auto element : message.payload) {
        message.checksum_a += element;
        message.checksum_b += message.checksum_a;
    }
}

