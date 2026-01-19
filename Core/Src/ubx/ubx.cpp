#include "ubx/ubx.hpp"

/**
 * @brief   Calculate the checksum values (checksumA and checksumB) for a UBX message
 * @param 	Message to calculate the checksum for
 */
void UBX::compute_checksum(UBX_message &message)
{
    message.checksumA = static_cast<uint8_t>(message.msgClass);
    message.checksumB = message.checksumA;

    message.checksumA += message.msgID;
    message.checksumB += message.checksumA;

    message.checksumA += message.payloadLength % (1 << 8);
    message.checksumB += message.checksumA;

    message.checksumA += message.payloadLength >> 8;
    message.checksumB += message.checksumA;

    for (int i = 0; i < message.payloadLength; i++) {
        message.checksumA += message.payload.at(i);
        message.checksumB += message.checksumA;
    }
}

