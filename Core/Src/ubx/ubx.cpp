#include "ubx/ubx.hpp"

/**
 * @brief   Calculate the checksum values (checksumA and checksumB) for a UBX message
 * @param 	Message to calculate the checksum for
 */
void UBX::ComputeChecksum(UbxMessage& message)
{
	message.checksumA = message.msgClass;
	message.checksumB = message.checksumA;

	message.checksumA += message.msgId;
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

