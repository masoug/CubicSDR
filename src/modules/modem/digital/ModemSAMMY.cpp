#include <sstream>

#include "ModemSAMMY.hpp"


#define OUTBUFFER_SIZE 16000000

void InitModeSDecoder(ModeSDecoder& decoder)
{
    // Allocate the various buffers used by Modes
    if ( ((decoder.icao_cache = (uint32_t *) malloc(sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2)                  ) == NULL) ||
         ((decoder.pFileData  = (uint16_t *) malloc(MODES_ASYNC_BUF_SIZE)                                         ) == NULL) ||
         ((decoder.magnitude  = (uint16_t *) malloc(MODES_ASYNC_BUF_SIZE+MODES_PREAMBLE_SIZE+MODES_LONG_MSG_SIZE) ) == NULL) ||
         ((decoder.maglut     = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL))
    {
        fprintf(stderr, "Out of memory allocating data buffer.\n");
        exit(1);
    }

    // Clear the buffers that have just been allocated, just in-case
    memset(decoder.icao_cache, 0,   sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2);
    memset(decoder.pFileData,127,   MODES_ASYNC_BUF_SIZE);
    memset(decoder.magnitude,  0,   MODES_ASYNC_BUF_SIZE+MODES_PREAMBLE_SIZE+MODES_LONG_MSG_SIZE);

    // Validate the users Lat/Lon home location inputs
    if ( (decoder.fUserLat >   90.0)  // Latitude must be -90 to +90
         || (decoder.fUserLat <  -90.0)  // and
         || (decoder.fUserLon >  360.0)  // Longitude must be -180 to +360
         || (decoder.fUserLon < -180.0) ) {
        decoder.fUserLat = decoder.fUserLon = 0.0;
    } else if (decoder.fUserLon > 180.0) { // If Longitude is +180 to +360, make it -180 to 0
        decoder.fUserLon -= 360.0;
    }
    // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the
    // Atlantic ocean off the west coast of Africa. This is unlikely to be correct.
    // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian
    // is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both.
    // Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
    decoder.bUserFlags &= ~MODES_USER_LATLON_VALID;
    if ((decoder.fUserLat != 0.0) || (decoder.fUserLon != 0.0)) {
        decoder.bUserFlags |= MODES_USER_LATLON_VALID;
    }

    // Initialise the Block Timers to something half sensible
    ftime(&decoder.stSystemTimeBlk);
    for (int i = 0; i < MODES_ASYNC_BUF_NUMBER; i++)
    {decoder.stSystemTimeRTL[i] = decoder.stSystemTimeBlk;}

    // Each I and Q value varies from 0 to 255, which represents a range from -1 to +1. To get from the
    // unsigned (0-255) range you therefore subtract 127 (or 128 or 127.5) from each I and Q, giving you
    // a range from -127 to +128 (or -128 to +127, or -127.5 to +127.5)..
    //
    // To decode the AM signal, you need the magnitude of the waveform, which is given by sqrt((I^2)+(Q^2))
    // The most this could be is if I&Q are both 128 (or 127 or 127.5), so you could end up with a magnitude
    // of 181.019 (or 179.605, or 180.312)
    //
    // However, in reality the magnitude of the signal should never exceed the range -1 to +1, because the
    // values are I = rCos(w) and Q = rSin(w). Therefore the integer computed magnitude should (can?) never
    // exceed 128 (or 127, or 127.5 or whatever)
    //
    // If we scale up the results so that they range from 0 to 65535 (16 bits) then we need to multiply
    // by 511.99, (or 516.02 or 514). antirez's original code multiplies by 360, presumably because he's
    // assuming the maximim calculated amplitude is 181.019, and (181.019 * 360) = 65166.
    //
    // So lets see if we can improve things by subtracting 127.5, Well in integer arithmatic we can't
    // subtract half, so, we'll double everything up and subtract one, and then compensate for the doubling
    // in the multiplier at the end.
    //
    // If we do this we can never have I or Q equal to 0 - they can only be as small as +/- 1.
    // This gives us a minimum magnitude of root 2 (0.707), so the dynamic range becomes (1.414-255). This
    // also affects our scaling value, which is now 65535/(255 - 1.414), or 258.433254
    //
    // The sums then become mag = 258.433254 * (sqrt((I*2-255)^2 + (Q*2-255)^2) - 1.414)
    //                   or mag = (258.433254 * sqrt((I*2-255)^2 + (Q*2-255)^2)) - 365.4798
    //
    // We also need to clip mag just incaes any rogue I/Q values somehow do have a magnitude greater than 255.
    //

    for (int i = 0; i <= 255; i++) {
        for (int q = 0; q <= 255; q++) {
            int mag, mag_i, mag_q;

            mag_i = (i * 2) - 255;
            mag_q = (q * 2) - 255;

            mag = (int) round((sqrt((mag_i*mag_i)+(mag_q*mag_q)) * 258.433254) - 365.4798);

            decoder.maglut[(i*256)+q] = (uint16_t) ((mag < 65535) ? mag : 65535);
        }
    }

    // set debug flags for now
    decoder.stats = 1;
    decoder.debug |= MODES_DEBUG_DEMOD;
    decoder.debug |= MODES_DEBUG_DEMODERR;
    decoder.debug |= MODES_DEBUG_GOODCRC;
    decoder.debug |= MODES_DEBUG_BADCRC;
    decoder.debug |= MODES_DEBUG_NOPREAMBLE;
//    decoder.debug = 0;
}

void FreeModeSDecoder(ModeSDecoder& decoder)
{
    free(decoder.icao_cache);
    free(decoder.pFileData);
    free(decoder.magnitude);
    free(decoder.maglut);
}

static
std::string
write_stats(ModeSDecoder& decoder) {
    int j;
    time_t now = time(NULL);

    std::stringstream result;

    result << "\n\n";
    result << "Statistics as at " << ctime(&now) << std::endl;

    result << decoder.stat_blocks_processed << " sample blocks processed\n";
    result << decoder.stat_blocks_dropped << " sample blocks dropped\n";
    result << decoder.stat_ModeAC << " ModeA/C detected\n";
    result << decoder.stat_valid_preamble << " valid Mode-S preambles\n";
    result << decoder.stat_DF_Len_Corrected << " DF-?? fields corrected for length\n";
    result << decoder.stat_DF_Type_Corrected << " DF-?? fields corrected for type\n";
    result << decoder.stat_demodulated0 << " demodulated with 0 errors\n";
    result << decoder.stat_demodulated1 << " demodulated with 1 error\n";
    result << decoder.stat_demodulated2 << " demodulated with 2 errors\n";
    result << decoder.stat_demodulated3 << " demodulated with > 2 errors\n";
    result << decoder.stat_goodcrc << " with good crc\n";
    result << decoder.stat_badcrc << " with bad crc\n";
    result << decoder.stat_fixed << " errors corrected\n";

    for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
        result << "   " << decoder.stat_bit_fix[j] << " with " << j+1 << " bit " << ((j==0)?"error":"errors") << "\n";
    }

    if (decoder.phase_enhance) {
        result << decoder.stat_out_of_phase << " phase enhancement attempts\n";
        result << decoder.stat_ph_demodulated0 << " phase enhanced demodulated with 0 errors\n";
        result << decoder.stat_ph_demodulated1 << " phase enhanced demodulated with 1 error\n";
        result << decoder.stat_ph_demodulated2 << " phase enhanced demodulated with 2 errors\n";
        result << decoder.stat_ph_demodulated3 << " phase enhanced demodulated with > 2 errors\n";
        result << decoder.stat_ph_goodcrc << " phase enhanced with good crc\n";
        result << decoder.stat_ph_badcrc << " phase enhanced with bad crc\n";
        result << decoder.stat_ph_fixed << " phase enhanced errors corrected\n";

        for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
            result << "   " << decoder.stat_ph_bit_fix[j] << " with " << j+1 << " bit " << ((j==0)? "error":"errors") << "\n";
        }
    }

    result << decoder.stat_goodcrc + decoder.stat_ph_goodcrc + decoder.stat_fixed + decoder.stat_ph_fixed << " total usable messages\n";

    decoder.stat_blocks_processed =
    decoder.stat_blocks_dropped = 0;

    decoder.stat_ModeAC =
    decoder.stat_valid_preamble =
    decoder.stat_DF_Len_Corrected =
    decoder.stat_DF_Type_Corrected =
    decoder.stat_demodulated0 =
    decoder.stat_demodulated1 =
    decoder.stat_demodulated2 =
    decoder.stat_demodulated3 =
    decoder.stat_goodcrc =
    decoder.stat_badcrc =
    decoder.stat_fixed = 0;

    decoder.stat_out_of_phase =
    decoder.stat_ph_demodulated0 =
    decoder.stat_ph_demodulated1 =
    decoder.stat_ph_demodulated2 =
    decoder.stat_ph_demodulated3 =
    decoder.stat_ph_goodcrc =
    decoder.stat_ph_badcrc =
    decoder.stat_ph_fixed = 0;

    for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
        decoder.stat_ph_bit_fix[j] = 0;
        decoder.stat_bit_fix[j] = 0;
    }

    return result.str();
}

ModemSAMMY::ModemSAMMY()
    : ModemDigital(), m_decoder(), m_buffer(MODES_ASYNC_BUF_SAMPLES), m_stats_count(0),
    m_outfile("modem_sammy.dat", std::ios::binary),
    m_outbuffer(OUTBUFFER_SIZE), m_out_count(0)
{
    std::cout << "ModemSAMMY::ModemSAMMY()" << std::endl;

//    InitModeSDecoder(m_decoder);
//    modesInitErrorInfo(&m_decoder);
}

ModemSAMMY::~ModemSAMMY()
{
    std::cout << "ModemSAMMY::~ModemSAMMY()" << std::endl;

//    FreeModeSDecoder(m_decoder);
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
    return MODES_DEFAULT_RATE;
}

int ModemSAMMY::getDefaultSampleRate() {
    return MODES_DEFAULT_RATE;
}

void ModemSAMMY::demodulate(ModemKit *kit, ModemIQData *input, AudioThreadInput *audioOut)
{
    auto* dkit = (ModemKitDigital *)kit;
    digitalStart(dkit, nullptr, input);

//    if (input->data.size() > m_buffer.size()) {
//        m_buffer.resize(input->data.size());
//    }

    for (size_t i = 0; i < input->data.size(); i++) {
        const float real = input->data[i].real;
        const float imag = input->data[i].imag;
        const float mag = std::sqrt((real*real) + (imag*imag));

//        m_buffer[i] = static_cast<uint16_t>(10000.0 * mag);
        if (m_out_count < OUTBUFFER_SIZE) {
            if (m_out_count % 100000 == 0) {
                outStream << "Filling buffer... " << m_out_count << std::endl;
            }
            m_outbuffer[m_out_count] = mag;
            m_out_count++;
        } else if (m_out_count == OUTBUFFER_SIZE) {
            outStream << "Saving buffer..." << std::endl;
            m_outfile.write((char*)m_outbuffer.data(), m_outbuffer.size() * sizeof(float));
            m_outfile.flush();
            m_out_count++;
        }
    }

//    detectModeS(&m_decoder, m_buffer.data(), static_cast<uint32_t>(input->data.size()));
//
//    if (m_stats_count == 99) {
//        outStream << write_stats(m_decoder) << std::endl;
//        m_stats_count = 0;
//    } else {
//        m_stats_count++;
//    }

    digitalFinish(dkit, nullptr);
}



