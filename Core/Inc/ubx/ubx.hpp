/*
 UBX MESSAGE STRUCTURE:
 each part of the message consist of 1 byte, the length of the payload
 (only the payload) is an unsigned 16 bit integer (little endian)
 sync char 1 | sync char 2 | class | id | length of payload (2 byte little endian) | payload | checksum A | checksum B

 NUMBER FORMATS:
 All multi-byte values are ordered in Little Endian format, unless
 otherwise indicated. All floating point values are transmitted in
 IEEE754 single or double precision.

 UBX CHECKSUM:
 The checksum is calculated over the Message, starting and including
 the CLASS field, up until, but excluding, the Checksum Field.
 The checksum algorithm used is the 8-Bit Fletcher Algorithm:

 NAV 0x01 Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
 RXM 0x02 Receiver Manager Messages: Satellite Status, RTC Status
 INF 0x04 Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
 ACK 0x05 Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
 CFG 0x06 Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
 UPD 0x09 Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
 MON 0x0A Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
 AID 0x0B AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
 TIM 0x0D Timing Messages: Time Pulse Output, Time Mark Results
 ESF 0x10 External Sensor Fusion Messages: External Sensor Measurements and Status Information
 MGA 0x13 Multiple GNSS Assistance Messages: Assistance data for various GNSS
 LOG 0x21 Logging Messages: Log creation, deletion, info and retrieval
 SEC 0x27 Security Feature Messages
 HNR 0x28 High Rate Navigation Results Messages: High rate time, position, speed, heading
 */

#ifndef UBX_HPP
#define UBX_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <array>

//******* DEBUG/HELPING CONSTANTS ********
static constexpr int MAX_MESSAGE_LENGTH = 1000;
static constexpr int MAX_PAYLOAD_LENGTH = 92;


struct alignas(128) UbxMessage {
	uint8_t sync1;
	uint8_t sync2;
	uint8_t msgClass;
	uint8_t msgId;
	uint16_t payloadLength;
	std::array<uint8_t, MAX_PAYLOAD_LENGTH> payload;
	uint8_t checksumA;
	uint8_t checksumB;
};

struct alignas(2) MessageInfo {
	uint8_t msgClass;
	uint8_t msgId;
};

class UBX {
public:
	UBX();
	~UBX();
	void ComposeMessage(const MessageInfo &messageInfo,
			const std::vector<uint8_t> &payload);
	const UbxMessage& GetUBXMessage() const {
		return message_;
	}
	void ComputeChecksum();
	void ResetPayload();

	//********* SYNC CHAR SECTION **********
	static constexpr uint8_t SYNC_CHAR_1 = 0xB5;
	static constexpr uint8_t SYNC_CHAR_2 = 0x62;

	//********* NAV MESSAGE SECTION **********
	static constexpr uint8_t NAV_PVT = 0x07;

	//********* ACK MESSAGE SECTION **********
	static constexpr uint8_t ACK_ACK = 0x01;
	static constexpr uint8_t ACK_NAK = 0x00;

	//********* CFG MESSAGE SECTION **********
	static constexpr uint8_t CFG_PRT = 0x00;
	static constexpr uint8_t CFG_MSG = 0x01;
	static constexpr uint8_t CFG_RATE = 0x08;

	enum class MsgClass {
		NAV_CLASS = 0x01,
		RXM_CLASS = 0x02,
		INF_CLASS = 0x04,
		ACK_CLASS = 0x05,
		CFG_CLASS = 0x06,
		UPD_CLASS = 0x09,
		MON_CLASS = 0x0A,
		AID_CLASS = 0x0B,
		TIM_CLASS = 0x0D,
		ESF_CLASS = 0x10,
		MGA_CLASS = 0x13,
		LOG_CLASS = 0x21,
		SEC_CLASS = 0x27,
		HNR_CLASS = 0x28,
	};

	enum class FixFlags {
		NO_FIX = 0,
		DEAD_RECKONING_ONLY = 1,
		TWO_D_FIX = 2,
		THREE_D_FIX = 3,
		GNSS_DEAD_RECKONING_COMBINED = 4,
		TIME_ONLY_FIX = 5,
	};

	std::string MsgClassToString(MsgClass msgClass) const;
	std::string GetGNSSFixType(FixFlags fixFlag) const;
	std::string UbxMessageToString() const;

private:
	UbxMessage message_;
};

#endif // UBX_HPP
