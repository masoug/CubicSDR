// Copyright (c) Charles J. Cliffe
// SPDX-License-Identifier: GPL-2.0+

#pragma once

#include <map>
#include <vector>
#include <sstream>
#include <ostream>
#include <mutex>

#include "DigitalConsoleFrame.h"
#include "ADSBFrame.hpp"
#include "ModemDigital.h"


class ModemDigitalOutputConsole;

class ADSBConsole: public ADSBFrame {
public:
    ADSBConsole(wxWindow* parent, ModemDigitalOutputConsole* modem_parent);
    ~ADSBConsole() override;

    void push_frame(const adsb_decoder::ByteArray& frame_bytes);
    void reset_stats();

private:

    struct TrackedAircraft {
        wxString icao{};
        wxString callsign{};

        uint32_t last_updated{};

        void update(const adsb_decoder::ModeSMessage& msg);
    };

    void DoRefresh(wxTimerEvent& event) override;
    void OnClose(wxCloseEvent& evt) override;


    ModemDigitalOutputConsole* m_modem_parent;

    unsigned int m_msgs_recv;

    std::list<adsb_decoder::ByteArray> m_raw_frame_queue;
    std::mutex m_frame_queue_mutex;
    std::map<int, TrackedAircraft> m_aircraft_manager;
};

class DigitalConsole: public DigitalConsoleFrame {
public:
    DigitalConsole( wxWindow* parent, ModemDigitalOutputConsole *doParent );
    ~DigitalConsole();


    void write(std::string outp);
    void write(char outc);
    
private:
    void DoRefresh( wxTimerEvent& event );
    void OnClose( wxCloseEvent& event );
    void OnClear( wxCommandEvent& event );
    
    void OnCopy( wxCommandEvent& event );
    void OnPause( wxCommandEvent& event );

    std::stringstream streamBuf;
    std::mutex stream_busy;
    std::atomic<bool> streamWritten;
    std::atomic<bool> streamPaused;
    ModemDigitalOutputConsole *doParent;
};

class ModemDigitalOutputConsole: public ModemDigitalOutput {
public:
    ModemDigitalOutputConsole();
    ~ModemDigitalOutputConsole();
    
    void setDialog(DigitalConsole *dialog_in);
    void setDialog(ADSBConsole *dialog_in);
    DigitalConsole *getDialog();
    ADSBConsole *getADSBDialog();

    void setTitle(std::string title);
    
    void write(std::string outp);
    void write(char outc);
    
    void Show();
    void Hide();
    void Close();

private:
    DigitalConsole *dialog;
    ADSBConsole* adsb_console;
    std::stringstream streamBuf;
    std::mutex stream_busy;
    std::atomic<bool> streamWritten;
    std::string dialogTitle;
};

