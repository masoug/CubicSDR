#ifndef CUBICSDR_MODEMSAMMY_HPP
#define CUBICSDR_MODEMSAMMY_HPP

#include <fstream>
#include <array>

#include "ModemDigital.h"


class ModemSAMMY : public ModemDigital {
public:

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
