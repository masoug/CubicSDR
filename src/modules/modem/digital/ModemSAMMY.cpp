#include <sstream>
#include <iomanip>

#include "dump1090.h"
#include "ModemSAMMY.hpp"
#include "ADSBDecoder.hpp"


ModemSAMMY::ModemSAMMY()
    : ModemDigital(),
      m_buffer(adsb_decoder::BUFFER_SIZE), m_buf_idx(0)
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
    return adsb_decoder::SAMPLE_RATE;
}

int ModemSAMMY::getDefaultSampleRate() {
    return adsb_decoder::SAMPLE_RATE;
}

void
ModemSAMMY::_decode_mode_s()
{
    using namespace adsb_decoder;

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

        // and check if that bytestream is valid data by applying the CRC check
        const auto checksum = compute_checksum(frame_bitvector);
        if (checksum == 0) {
            valid_messages++;
        } else {
            continue;
        }

        // if the CRC check passes, pass the data along the bytestream
        const ByteArray frame_bytes = extract_bytearray(frame_bitvector);
        outStream.write((char*)frame_bytes.data(), frame_bytes.size());
//        const auto message = extract_message(frame_bytes);
//
//        outStream << message.to_string() << std::endl << std::endl;

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
    if (m_buf_idx >= adsb_decoder::BUFFER_SIZE)
    {
//        std::cout << "Decoding buffer... m_buf_idx=" << m_buf_idx << std::endl;
        // scan through the buffer and extract any messages that can be decoded
        _decode_mode_s();

        // reset the buffer, copying over any remaining data from the end of the buffer
        // to the front, and continue filling
        std::copy(m_buffer.begin() + adsb_decoder::BUFFER_THRESHOLD, m_buffer.begin() + m_buf_idx, m_buffer.begin());
        m_buf_idx -= adsb_decoder::BUFFER_THRESHOLD;

//        std::cout << "  New m_buf_idx=" << m_buf_idx << std::endl;
//        std::cout << std::endl;
    }

    digitalFinish(dkit, nullptr);
}