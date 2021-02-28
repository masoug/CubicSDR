#ifndef CUBICSDR_ADSBDECODER_HPP
#define CUBICSDR_ADSBDECODER_HPP

#include <array>
#include <string>

namespace adsb_decoder {

static constexpr unsigned int SAMPLE_RATE = 2000000;
static constexpr unsigned int BITS_PER_FRAME = 112;
static constexpr unsigned int BYTES_PER_FRAME = BITS_PER_FRAME / 8;
static constexpr unsigned int BITS_PER_PREAMBLE = 8;
static constexpr unsigned int SAMPLES_PER_FRAME = 2 * BITS_PER_FRAME;
static constexpr unsigned int SAMPLES_PER_PREAMBLE = 2 * BITS_PER_PREAMBLE;
static constexpr unsigned int SAMPLES_PER_PACKET = SAMPLES_PER_PREAMBLE + SAMPLES_PER_FRAME;
static constexpr unsigned int BUFFER_THRESHOLD = 1048576;
static constexpr unsigned int BUFFER_SIZE = BUFFER_THRESHOLD + SAMPLES_PER_PACKET;

using BitVector = std::array<bool, BITS_PER_FRAME>;
using ByteArray = std::array<uint8_t, BYTES_PER_FRAME>;
using MessageArray = std::array<uint8_t, 7>;


enum ADSBMessageType {
    CALLSIGN,
    UNDEFINED
};

class ADSBMessage {
public:

    static std::shared_ptr<ADSBMessage>
    ADSBMessageFactory(int type_code, const MessageArray &message_array);

    virtual std::string to_string() = 0;
    virtual ADSBMessageType message_type() const { return UNDEFINED; }
};

class ADSBAircraftID : public ADSBMessage {
public:
    explicit ADSBAircraftID(const MessageArray &message_bytes);

    std::string to_string() override;

    std::string callsign() const { return m_callsign; }
    ADSBMessageType message_type() const override {return CALLSIGN; }

private:
    std::string m_callsign;
};

struct ModeSMessage {
    ByteArray frame_bytes{};
    int downlink_format{0};
    int capability{0};
    int icao{0};
    MessageArray message_bytes{};
    int type_code{0};
    std::shared_ptr<ADSBMessage> message{nullptr};

    std::string to_string() const;
};


bool
check_preamble_relations(const float* buffer);

bool
check_preamble_levels(const float* buffer);

BitVector
extract_bitvector(const float *buffer);

ByteArray
extract_bytearray(const BitVector& bitvector);

uint32_t
compute_checksum(const BitVector& bitvector);

ModeSMessage
extract_message(const ByteArray& frame_bytes);

}

#endif //CUBICSDR_ADSBDECODER_HPP
