#include <sstream>
#include <iomanip>

#include "dump1090.h"
#include "ModemSAMMY.hpp"


using BitVector = std::array<bool, ModemSAMMY::BITS_PER_FRAME>;

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


inline std::string
to_hex(const uint8_t byte) {
    std::string result(2, '0');

    result[0] = dec2hex[byte >> 4];
    result[1] = dec2hex[byte & 0x0F];

    return result;
}

std::string
ModemSAMMY::ModeSMessage::to_string() const
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
    for (const auto& b : message) {
        ss << to_hex(b);
    }
    ss << std::endl;

    ss << "  Type Code: " << type_code << std::endl;

    return ss.str();
}


ModemSAMMY::ModemSAMMY()
    : ModemDigital(),
      m_buffer(BUFFER_SIZE), m_buf_idx(0)
//    m_stats_count(0), m_outfile("modem_sammy.dat", std::ios::binary), m_outbuffer(OUTBUFFER_SIZE), m_out_count(0)
{
    std::cout << "ModemSAMMY::ModemSAMMY()" << std::endl;
}

ModemSAMMY::~ModemSAMMY()
{
    std::cout << "ModemSAMMY::~ModemSAMMY()" << std::endl;
}

std::string ModemSAMMY::getName()
{
    return "SAMMY";
}

ModemBase *ModemSAMMY::factory()
{
    return new ModemSAMMY;
}

int ModemSAMMY::checkSampleRate(long long, int) {
    return SAMPLE_RATE;
}

int ModemSAMMY::getDefaultSampleRate() {
    return SAMPLE_RATE;
}

inline bool
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

inline bool
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

inline BitVector
extract_bitvector(const float *buffer)
{
    BitVector bitvector{};
    for (size_t i = 0; i < ModemSAMMY::BITS_PER_FRAME; i++) {
        bitvector[i] = buffer[2*i] > buffer[(2*i)+1];
    }

    return bitvector;
}

inline ModemSAMMY::ByteArray
extract_bytearray(const BitVector& bitvector)
{
    ModemSAMMY::ByteArray result{};
    for (size_t i = 0; i < ModemSAMMY::BYTES_PER_FRAME; i++) {
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

ModemSAMMY::ModeSMessage
extract_message(const ModemSAMMY::ByteArray& frame_bytes)
{
    ModemSAMMY::ModeSMessage message{};
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
    std::copy(frame_bytes.begin()+4, frame_bytes.begin()+11, message.message.begin());

    return message;
}

void
ModemSAMMY::_decode_mode_s()
{
    // scan through the buffer
    unsigned int possible_preambles = 0;
    unsigned int valid_messages = 0;
    for (int i = 0; i < BUFFER_THRESHOLD; i++)
    {
        // shamelessly borrowed from dump1090.c

        // The Mode S preamble is made of impulses of 0.5 microseconds at
        // the following time offsets:
        //
        // 0   - 0.5 usec: first impulse.
        // 1.0 - 1.5 usec: second impulse.
        // 3.5 - 4   usec: third impulse.
        // 4.5 - 5   usec: last impulse.
        //
        // Since we are sampling at 2 Mhz every sample in our magnitude vector
        // is 0.5 usec, so the preamble will look like this, assuming there is
        // an impulse at offset 0 in the array:
        //
        // 0   -----------------
        // 1   -
        // 2   ------------------
        // 3   --
        // 4   -
        // 5   --
        // 6   -
        // 7   ------------------
        // 8   --
        // 9   -------------------
        if (not check_preamble_relations(&m_buffer[i]))
        {
            // samples at the current position in the buffer don't match the pattern of a preamble
            continue;
        }

        if (not check_preamble_levels(&m_buffer[i]))
        {
            // levels in between the spikes in the buffer don't match the profile for a preamble
            continue;
        }

        possible_preambles++;

        // now that we have a possible preamble, extract the frame from the buffer into a bytestream
        const auto frame_bitvector = extract_bitvector(&m_buffer[i + SAMPLES_PER_PREAMBLE]);
        const ByteArray frame_bytes = extract_bytearray(frame_bitvector);

        // and check if that bytestream is valid data by applying the CRC check
        const auto checksum = compute_checksum(frame_bitvector);
//        const auto checksum = modesChecksum((unsigned char*)frame_bytes.data(), 112);
//        if (checksum != checksum1) {
//            std::cout << "Checksum mismatch: Expect=" << checksum << " got=" << checksum1 << std::endl;
//        }
        if (checksum == 0) {
            valid_messages++;
        } else {
            continue;
        }

        // if the CRC check passes, extract the data...
        const auto message = extract_message(frame_bytes);

        outStream << message.to_string() << std::endl << std::endl;

        // ...then jump to the next sample after the end of this packet to start scanning for the next preamble
        i += SAMPLES_PER_FRAME;
    }

//    std::cout << "Decoder Report:" << std::endl;
//    std::cout << "  " << possible_preambles << " possible preambles." << std::endl;
//    std::cout << "  " << valid_messages << " valid messages." << std::endl;
//    std::cout << ::std::endl;
}

void ModemSAMMY::demodulate(ModemKit *kit, ModemIQData *input, AudioThreadInput *audioOut)
{
    auto* dkit = (ModemKitDigital *)kit;
    digitalStart(dkit, nullptr, input);

    const auto bufsize = m_buf_idx + input->data.size() + 1;
    if (bufsize > m_buffer.size()) {
        m_buffer.resize(bufsize);
    }

    for (auto &sample : input->data) {
        const float real = sample.real;
        const float imag = sample.imag;
        const float mag = std::sqrt((real*real) + (imag*imag));

        m_buffer[m_buf_idx] = mag;
        m_buf_idx++;

//        if (m_out_count < OUTBUFFER_SIZE) {
//            if (m_out_count % 100000 == 0) {
//                outStream << "Filling buffer... " << m_out_count << std::endl;
//            }
//            m_outbuffer[m_out_count] = mag;
//            m_out_count++;
//        } else if (m_out_count == OUTBUFFER_SIZE) {
//            outStream << "Saving buffer..." << std::endl;
//            m_outfile.write((char*)m_outbuffer.data(), m_outbuffer.size() * sizeof(float));
//            m_outfile.flush();
//            m_out_count++;
//        }
    }

    // once we've filled the buffer we want to have the decoder scan through and identify messages.
    if (m_buf_idx >= BUFFER_SIZE)
    {
//        std::cout << "Decoding buffer... m_buf_idx=" << m_buf_idx << std::endl;
        // scan through the buffer and extract any messages that can be decoded
        _decode_mode_s();

        // reset the buffer, copying over any remaining data from the end of the buffer
        // to the front, and continue filling
        std::copy(m_buffer.begin() + BUFFER_THRESHOLD, m_buffer.begin() + m_buf_idx, m_buffer.begin());
        m_buf_idx -= BUFFER_THRESHOLD;

//        std::cout << "  New m_buf_idx=" << m_buf_idx << std::endl;
//        std::cout << std::endl;
    }

    digitalFinish(dkit, nullptr);
}



