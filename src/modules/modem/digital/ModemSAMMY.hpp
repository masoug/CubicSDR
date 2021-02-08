#ifndef CUBICSDR_MODEMSAMMY_HPP
#define CUBICSDR_MODEMSAMMY_HPP


#include <fstream>
#include <array>

#include "ModemDigital.h"

class ModemSAMMY : public ModemDigital {
public:
    static constexpr unsigned int SAMPLE_RATE = 2000000;
    static constexpr unsigned int BITS_PER_FRAME = 112;
    static constexpr unsigned int BYTES_PER_FRAME = BITS_PER_FRAME / 8;
    static constexpr unsigned int BITS_PER_PREAMBLE = 8;
    static constexpr unsigned int SAMPLES_PER_FRAME = 2 * BITS_PER_FRAME;
    static constexpr unsigned int SAMPLES_PER_PREAMBLE = 2 * BITS_PER_PREAMBLE;
    static constexpr unsigned int SAMPLES_PER_PACKET = SAMPLES_PER_PREAMBLE + SAMPLES_PER_FRAME;
    static constexpr unsigned int BUFFER_THRESHOLD = 1048576;
    static constexpr unsigned int BUFFER_SIZE = BUFFER_THRESHOLD + SAMPLES_PER_PACKET;

    using ByteArray = std::array<uint8_t, BYTES_PER_FRAME>;
    using MessageArray = std::array<uint8_t, 7>;

    struct ModeSMessage {
        ByteArray frame_bytes{};
        int downlink_format{0};
        int capability{0};
        int icao{0};
        MessageArray message{};
        int type_code{0};

        std::string to_string() const;
    };

    ModemSAMMY();
    ~ModemSAMMY() override;

    std::string getName() override;

    static ModemBase *factory();

    int checkSampleRate(long long, int) override;
    int getDefaultSampleRate() override;

    void demodulate(ModemKit *kit, ModemIQData *input, AudioThreadInput *audioOut) override;

private:

    void _decode_mode_s();

    std::vector<float> m_buffer;
    unsigned int m_buf_idx;

//    unsigned int m_stats_count;
//    std::ofstream m_outfile;
//    std::vector<float> m_outbuffer;
//    size_t m_out_count;
};


#endif //CUBICSDR_MODEMSAMMY_HPP
