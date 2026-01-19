#include "ubx/ubx.hpp"

/**
 * @brief   Compose a UBX message.
 * @param   messageInfo   The message class and ID of the UBX message.
 * @param   payload       The payload data (default is nullptr).
 * @return  The composed UBX message.
 */
void UBX::ComposeMessage(const MessageInfo &messageInfo,
	const std::vector<uint8_t> &payload)
{
	ResetPayload();

	message_.sync1 = sync_char_1;
	message_.sync2 = sync_char_2;
	message_.msgClass = messageInfo.msgClass;
	message_.msgId = messageInfo.msgId;
	message_.payloadLength = std::min(static_cast<uint16_t>(payload.size()),
			static_cast<uint16_t>(MAX_PAYLOAD_LENGTH));

	std::copy(payload.begin(), payload.begin() + message_.payloadLength,
			std::begin(message_.payload));

	ComputeChecksum();
}

/**
 * @brief   Convert a message class (msgClass) to its corresponding string representations
 * @param   msgClass    The message class to convert.
 * @return  The string representation of the message class.
 */
std::string UBX::MsgClassToString(MsgClass msgClass) const {
	switch (msgClass) {
	case MsgClass::nav:
		return "Navigation";
	case MsgClass::rxm:
		return "Receiver Manager";
	case MsgClass::inf:
		return "Information";
	case MsgClass::ack:
		return "ACK/NAK";
	case MsgClass::cfg:
		return "Configuration";
	case MsgClass::upd:
		return "Firmware update";
	case MsgClass::mon:
		return "Monitoring";
	case MsgClass::aid:
		return "AssistNow messages";
	case MsgClass::tim:
		return "Timing";
	case MsgClass::esf:
		return "External Sensor Fusion Messages";
	case MsgClass::mga:
		return "Multiple GNSS Assistance Messages";
	case MsgClass::log:
		return "Logging";
	case MsgClass::sec:
		return "Security";
	case MsgClass::hnr:
		return "High rate navigation results";
	default:
		return "Couldn't find class";
	}
}

/**
 * @brief   Convert a GNSS fix flag (fixFlag) to its corresponding string representation.
 * @param   fixFlag The GNSS fix flag to convert.
 * @return  The string representation of the GNSS fix type.
 */
std::string UBX::GetGNSSFixType(FixFlags fixFlag) const {
	switch (fixFlag) {
	case FixFlags::no_fix:
		return "No fix";
	case FixFlags::dead_reckoning_only:
		return "Dead reckoning only";
	case FixFlags::two_d_fix:
		return "2D-fix";
	case FixFlags::three_d_fix:
		return "3D-fix";
	case FixFlags::gnss_dead_reckoning_combined:
		return "GNSS + dead reckoning combined";
	case FixFlags::time_only_fix:
		return "Time only fix";
	default:
		return "Reserved / No fix";
	}
}

/**
 * @brief   Calculate the checksum values (checksumA and checksumB) for a UBX message.
 */
void UBX::ComputeChecksum()
{
  message_.checksumA = message_.msgClass;
  message_.checksumB = message_.checksumA;

  message_.checksumA += message_.msgId;
  message_.checksumB += message_.checksumA;

  message_.checksumA += message_.payloadLength % (1 << 8);
  message_.checksumB += message_.checksumA;

  message_.checksumA += message_.payloadLength >> 8;
  message_.checksumB += message_.checksumA;

  for (int i = 0; i < message_.payloadLength; i++) {
    message_.checksumA += message_.payload.at(i);
    message_.checksumB += message_.checksumA;
  }
}

/**
 * @brief   Reset the payload of a UBX message to all zeros.
 */
void UBX::ResetPayload() {
	std::fill(message_.payload.begin(), message_.payload.end(), 0);
}
