#include <sstream>
#include <iomanip>

#include "ADSBDecoder.hpp"

namespace adsb_decoder {

// Borrowed from "dump1090" project
// ===================== Mode S detection and decoding  ===================
//
// Parity table for MODE S Messages.
// The table contains 112 elements, every element corresponds to a bit set
// in the message, starting from the first bit of actual data after the
// preamble.
//
// For messages of 112 bit, the whole table is used.
// For messages of 56 bits only the last 56 elements are used.
//
// The algorithm is as simple as xoring all the elements in this table
// for which the corresponding bit on the message is set to 1.
//
// The latest 24 elements in this table are set to 0 as the checksum at the
// end of the message should not affect the computation.
//
// Note: this function can be used with DF11 and DF17, other modes have
// the CRC xored with the sender address as they are reply to interrogations,
// but a casual listener can't split the address from the checksum.
//
constexpr uint32_t modes_checksum_table[112] = {
        0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
        0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
        0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
        0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
        0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
        0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
        0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
        0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
        0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
        0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
        0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
        0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
        0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
        0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

constexpr char dec2hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
constexpr char callsign_map[] =  "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";


bool
check_preamble_relations(const float* buffer)
{
    // First check of relations between the first 10 samples
    // representing a valid preamble. We don't even investigate further
    // if this simple test is not passed

    return buffer[0] > buffer[1] and
           buffer[1] < buffer[2] and
           buffer[2] > buffer[3] and
           buffer[3] < buffer[0] and
           buffer[4] < buffer[0] and
           buffer[5] < buffer[0] and
           buffer[6] < buffer[0] and
           buffer[7] > buffer[8] and
           buffer[8] < buffer[9] and
           buffer[9] > buffer[6];
}

bool
check_preamble_levels(const float* buffer)
{
    // The samples between the two spikes must be < than the average
    // of the high spikes level. We don't test bits too near to
    // the high levels as signals can be out of phase so part of the
    // energy can be in the near samples
    const float high = (buffer[0] + buffer[2] + buffer[7] + buffer[9]) / 6.f;
    if (buffer[4] >= high or buffer[5] >= high)
    {
        return false;
    }

    // Similarly samples in the range 11-14 must be low, as it is the
    // space between the preamble and real data. Again we don't test
    // bits too near to high levels, see above
    return buffer[11] < high and
           buffer[12] < high and
           buffer[13] < high and
           buffer[14] < high;
}

BitVector
extract_bitvector(const float *buffer)
{
    BitVector bitvector{};
    for (size_t i = 0; i < BITS_PER_FRAME; i++) {
        bitvector[i] = buffer[2*i] > buffer[(2*i)+1];
    }

    return bitvector;
}

ByteArray
extract_bytearray(const BitVector& bitvector)
{
    ByteArray result{};
    for (size_t i = 0; i < BYTES_PER_FRAME; i++) {
        const auto offset = i * 8;
        result[i]  = static_cast<uint8_t>(bitvector[offset + 0]) << 7;
        result[i] |= static_cast<uint8_t>(bitvector[offset + 1]) << 6;
        result[i] |= static_cast<uint8_t>(bitvector[offset + 2]) << 5;
        result[i] |= static_cast<uint8_t>(bitvector[offset + 3]) << 4;
        result[i] |= static_cast<uint8_t>(bitvector[offset + 4]) << 3;
        result[i] |= static_cast<uint8_t>(bitvector[offset + 5]) << 2;
        result[i] |= static_cast<uint8_t>(bitvector[offset + 6]) << 1;
        result[i] |= static_cast<uint8_t>(bitvector[offset + 7]) << 0;
    }

    return result;
}

uint32_t
compute_checksum(const BitVector& bitvector)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < bitvector.size() - 24; i++)
    {
        if (bitvector[i]) {
            crc ^= modes_checksum_table[i];
        }
    }

    uint32_t remainder = 0;
    for (size_t i = bitvector.size() - 24; i < bitvector.size(); i++)
    {
        remainder |= static_cast<uint32_t>(bitvector[i]);

        if (i < (bitvector.size() - 1)) {
            remainder <<= 1;
        }
    }
    const auto checksum = ((crc ^ remainder) & 0x00FFFFFF);
    return checksum;
}

ModeSMessage
extract_message(const ByteArray& frame_bytes)
{
    ModeSMessage message{};
    message.frame_bytes = frame_bytes;

    // the first 8 bits (1 byte) of the frame encodes the
    // downlink format (DF) and capabilities (CA) of the frame
    // DF is the first 5 bits while the CA is the last 3 bits
    const uint8_t df_ca_byte = frame_bytes[0];
    message.downlink_format = df_ca_byte >> 3;
    message.capability = df_ca_byte & 0x07;

    // the next 24 bits (3 bytes) encode the ICAO aircraft address
    message.icao = (frame_bytes[1] << 16) | (frame_bytes[2] << 8) | (frame_bytes[3]);

    // then find the type code that tells us how to further decode the message data
    const auto type_code = frame_bytes[4] >> 3;
    message.type_code = type_code;
    std::copy(frame_bytes.begin()+4, frame_bytes.begin()+11, message.message_bytes.begin());

    message.message = ADSBMessage::ADSBMessageFactory(type_code, message.message_bytes);

    return message;
}

inline std::string
to_hex(const uint8_t byte) {
    std::string result(2, '0');

    result[0] = dec2hex[byte >> 4];
    result[1] = dec2hex[byte & 0x0F];

    return result;
}


std::shared_ptr<ADSBMessage>
ADSBMessage::ADSBMessageFactory(
        const int               type_code,
        const MessageArray&     message_array)
{
    switch (type_code) {
        case 1:
        case 2:
        case 3:
        case 4:
            return std::make_shared<ADSBAircraftID>(message_array);
        default:
            return nullptr;
    }
}

ADSBAircraftID::ADSBAircraftID(const MessageArray &message_bytes)
        : m_callsign(8, '?')
{
    // decode the aircraft's callsign from the message array:
    // borrowed from https://mode-s.org/decode/content/ads-b/2-identification.html
    /**
     * In this message, the Type Code can be from 1 to 4.
     * The 56-bit ME filed consists of 10 parts and is structured as follows:
     * +------+------+------+------+------+------+------+------+------+------+
     * | TC,5 | CA,3 | C1,6 | C2,6 | C3,6 | C4,6 | C5,6 | C6,6 | C7,6 | C8,6 |
     * +------+------+------+------+------+------+------+------+------+------+
     * TC: Type code
     * CA: Aircraft category
     * C*: A character
     */

    uint64_t callsign_int{0};
    for (size_t i = 1; i < message_bytes.size(); i++)
    {
        callsign_int |= message_bytes[i];

        if (i < 6) {
            callsign_int <<= 8;
        }
    }

    for (size_t i = 0; i < 8; i++)
    {
        const auto bits2shift = 42 - (i * 6);
        const auto idx = static_cast<uint8_t>(0x000000000000003F & (callsign_int >> bits2shift));
        if (idx >= sizeof(callsign_map)) {
            m_callsign[i] = '?';
        } else {
            m_callsign[i] = callsign_map[idx];
        }
    }
}

std::string
ADSBAircraftID::to_string()
{
    std::stringstream ss;
    ss << "Aircraft Identification" << std::endl;
    ss << "  Callsign: " << callsign() << std::endl;
    return ss.str();
}

std::string
ModeSMessage::to_string() const
{
    std::stringstream ss;
    std::ios_base::fmtflags f(ss.flags());

    ss << "Raw Frame: ";
    for (const auto& b : frame_bytes) {
        ss << to_hex(b);
    }
    ss << std::endl;

    ss << "Downlink Format: " << downlink_format << std::endl;
    ss << "Capabilty: " << capability << std::endl;
    ss << "ICAO Address: " << std::setfill('0') << std::setw(6) << std::hex << std::uppercase << icao << std::endl;
    ss.flags(f);

    ss << "Raw Message: ";
    for (const auto& b : message_bytes) {
        ss << to_hex(b);
    }
    ss << std::endl;

    ss << "  Type Code: " << type_code << std::endl;
    ss << "******** ADSB Message ********" << std::endl;
    if (message) {
        ss << message->to_string() << std::endl;
    } else {
        ss << "UNKNOWN MESSAGE" << std::endl;
    }
    ss << "******************************" << std::endl;

    return ss.str();
}

}