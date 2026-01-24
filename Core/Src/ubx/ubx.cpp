#include "ubx/ubx.hpp"

namespace UBX {
    /**
     * @brief   Calculate the checksum values (checksumA and checksumB) for message
     * @param   Message to calculate the checksum for
     */
    Checksum compute_checksum(Message &message)
    {
        Checksum checksum{};
        checksum.a = static_cast<uint8_t>(message.header.msg_class);
        checksum.b = checksum.a;

        checksum.a += message.header.msg_id;
        checksum.b += checksum.a;

        checksum.a += message.header.payload_length % (1 << 8);
        checksum.b += checksum.a;

        checksum.a += message.header.payload_length >> 8;
        checksum.b += checksum.a;

        for (auto element : message.payload) {
            checksum.a += element;
            checksum.b += checksum.a;
        }

        return checksum;
    }
}


