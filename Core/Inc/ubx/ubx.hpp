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

class UBX
{
public:
    //********* SYNC CHAR SECTION **********
    static constexpr uint8_t sync_char_1 = 0xB5;
    static constexpr uint8_t sync_char_2 = 0x62;

    //********* NAV MESSAGE SECTION **********
    static constexpr uint8_t nav_pvt = 0x07;

    //********* ACK MESSAGE SECTION **********
    static constexpr uint8_t ack_ack = 0x01;
    static constexpr uint8_t ack_nak = 0x00;

    //********* CFG MESSAGE SECTION **********
    static constexpr uint8_t cfg_prt = 0x00;
    static constexpr uint8_t cfg_msg = 0x01;
    static constexpr uint8_t cfg_inf = 0x02;
    static constexpr uint8_t cfg_rate = 0x08;

    enum class MsgClass : uint8_t
    {
        nav = 0x01,
        rxm = 0x02,
        inf = 0x04,
        ack = 0x05,
        cfg = 0x06,
        upd = 0x09,
        mon = 0x0A,
        aid = 0x0B,
        tim = 0x0D,
        esf = 0x10,
        mga = 0x13,
        log = 0x21,
        sec = 0x27,
        hnr = 0x28,
    };

    enum class FixFlags : uint8_t
    {
        no_fix = 0,
        dead_reckoning_only = 1,
        two_d_fix = 2,
        three_d_fix = 3,
        gnss_dead_reckoning_combined = 4,
        time_only_fix = 5,
    };

    struct UbxMessage
    {
        uint8_t sync1
            { sync_char_1 };
        uint8_t sync2
            { sync_char_2 };
        MsgClass msgClass
            { MsgClass::nav };
        uint8_t msgID
            { cfg_prt };
        uint16_t payloadLength
            { 0 };
        std::vector<uint8_t> payload
            { };
        uint8_t checksumA
            { 0 };
        uint8_t checksumB
            { 0 };
    };

    void ComputeChecksum(UbxMessage &message);
};

#endif // UBX_HPP
