// Copyright (c) Charles J. Cliffe
// SPDX-License-Identifier: GPL-2.0+

#include "DigitalConsole.h"
#include "CubicSDR.h"
#include <iomanip>


ADSBConsole::ADSBConsole(
        wxWindow*                   parent,
        ModemDigitalOutputConsole*  modem_parent)
    : ADSBFrame(parent), m_modem_parent(modem_parent),
      m_msgs_recv(0), m_raw_frame_queue(), m_frame_queue_mutex(),
      m_aircraft_manager()
{
}

ADSBConsole::~ADSBConsole()
{
    m_modem_parent->setDialog((ADSBConsole*)nullptr);
}

void
ADSBConsole::push_frame(
        const adsb_decoder::ByteArray &frame_bytes)
{
    std::lock_guard<std::mutex> lg(m_frame_queue_mutex);
    if (m_raw_frame_queue.size() > 1000000) {
        return;
    }
    m_raw_frame_queue.push_back(frame_bytes);
}

void
ADSBConsole::reset_stats()
{
    m_msgs_recv = 0;
}

void
ADSBConsole::DoRefresh(
        wxTimerEvent& /* evt */)
{
    std::vector<adsb_decoder::ByteArray> frame_batch;

    {
        std::lock_guard<std::mutex> lg(m_frame_queue_mutex);

        if (m_raw_frame_queue.empty()) {
            return;
        }

        while (not m_raw_frame_queue.empty())
        {
            frame_batch.push_back(m_raw_frame_queue.front());
            m_raw_frame_queue.pop_front();
        }
    }

    // go through these batch of frames to decode and feed to the aircraft manager
    for (const auto& frame : frame_batch)
    {
        const auto message = adsb_decoder::extract_message(frame);
        std::cout << message.to_string() << std::endl;
        m_aircraft_manager[message.icao].update(message);
        m_msgs_recv++;
    }

    // TODO garbage-collect all aircraft that have timed out

    // TODO update gui with the new state from the aircraft manager
    const int missing_rows = static_cast<int>(m_aircraft_manager.size()) - m_aircraft_grid->GetNumberRows();
    if (missing_rows > 0) {
        m_aircraft_grid->AppendRows(missing_rows);
    }

    int row = 0;
    for (const auto& aircraft_entry : m_aircraft_manager)
    {
        const auto& aircraft = aircraft_entry.second;

        m_aircraft_grid->SetCellValue(row, 0, aircraft.icao);
        m_aircraft_grid->SetReadOnly(row, 0, true);
        m_aircraft_grid->SetCellValue(row, 1, aircraft.callsign);
        m_aircraft_grid->SetReadOnly(row, 1, true);

        row++;
    }
    for (; row < m_aircraft_grid->GetNumberRows(); row++) {
        for (int col = 0; col < GRID_WIDTH; col++) {
            m_aircraft_grid->SetCellValue(row, col, "");
        }
    }

    m_stats_text->SetLabel(wxString::Format(wxT("Received Messages: %i"), m_msgs_recv));
}

void
ADSBConsole::OnClose(wxCloseEvent &evt)
{
    m_modem_parent->setDialog((ADSBConsole*)nullptr);
}

void
ADSBConsole::TrackedAircraft::update(
        const adsb_decoder::ModeSMessage &msg)
{
    icao = wxString::Format("%06x", msg.icao).Upper();

    if (not msg.message) {
        return;
    }
    switch (msg.message->message_type()) {
        case adsb_decoder::ADSBMessageType::CALLSIGN:
            callsign = dynamic_cast<adsb_decoder::ADSBAircraftID*>(msg.message.get())->callsign();
            break;
        default:
            break;
    }
}


DigitalConsole::DigitalConsole( wxWindow* parent, ModemDigitalOutputConsole *doParent ): DigitalConsoleFrame( parent ), doParent(doParent) {
    streamWritten.store(false);
    streamPaused.store(false);
}

DigitalConsole::~DigitalConsole() {
    doParent->setDialog((DigitalConsole*)nullptr);
}

void DigitalConsole::OnClose( wxCloseEvent& /* event */ ) {
    doParent->setDialog((DigitalConsole*)nullptr);
}

void DigitalConsole::OnCopy( wxCommandEvent& /* event */ ) {
    m_dataView->SelectAll();
    m_dataView->Copy();
}

void DigitalConsole::OnPause( wxCommandEvent& /* event */ ) {
    if (streamPaused.load()) {
        m_pauseButton->SetLabel("Stop");
        streamPaused.store(false);
    } else {
        m_pauseButton->SetLabel("Run");
        streamPaused.store(true);
    }
}

void DoRefresh( wxTimerEvent& event ) {
    event.Skip();
}

void DigitalConsole::DoRefresh( wxTimerEvent& /* event */ ) {
    if (streamWritten.load()) {
        stream_busy.lock();
        m_dataView->AppendText(streamBuf.str());
        streamBuf.str("");
        streamWritten.store(false);
        stream_busy.unlock();
    }
}

void DigitalConsole::OnClear( wxCommandEvent& /* event */ ) {
    m_dataView->Clear();
}

void DigitalConsole::write(std::string outp) {
    if (streamPaused.load()) {
        return;
    }
    stream_busy.lock();
    streamBuf << outp;
    streamWritten.store(true);
    stream_busy.unlock();
}

void DigitalConsole::write(char outc) {
    if (streamPaused.load()) {
        return;
    }
    stream_busy.lock();
    streamBuf << outc;
    streamWritten.store(true);
    stream_busy.unlock();
}


ModemDigitalOutputConsole::ModemDigitalOutputConsole(): ModemDigitalOutput(), dialog(nullptr), adsb_console(nullptr) {
    streamWritten.store(false);
}

ModemDigitalOutputConsole::~ModemDigitalOutputConsole() {
    
}

void ModemDigitalOutputConsole::setDialog(DigitalConsole *dialog_in) {
    dialog = dialog_in;
    if (dialog && dialogTitle != "") {
        dialog->SetTitle(dialogTitle);
    }
}

void ModemDigitalOutputConsole::setDialog(ADSBConsole* dialog_in) {
    adsb_console = dialog_in;
    if (adsb_console && dialogTitle != "") {
        adsb_console->SetTitle(wxT("ADSB Decoder: ")+dialogTitle);
    }
}

DigitalConsole *ModemDigitalOutputConsole::getDialog() {
    return dialog;
}

ADSBConsole *ModemDigitalOutputConsole::getADSBDialog() {
    return adsb_console;
}

void ModemDigitalOutputConsole::Show() {
    if (dialog) {
        if (not dialog->IsShown()) {
            dialog->Show();
        }
    }

    if (adsb_console) {
        if (not adsb_console->IsShown()) {
            adsb_console->reset_stats();
            adsb_console->Show();
        }
    }
}

void ModemDigitalOutputConsole::Hide() {
    if (dialog) {
        if (dialog->IsShown()) {
            dialog->Hide();
        }
    }

    if (adsb_console) {
        if (adsb_console->IsShown()) {
            adsb_console->Hide();
        }
    }
}

void ModemDigitalOutputConsole::Close() {
    if (dialog) {
        dialog->Hide();
        dialog->Close();
        dialog = nullptr;
    }

    if (adsb_console) {
        adsb_console->Hide();
        adsb_console->Close();
        adsb_console = nullptr;
    }
}

void ModemDigitalOutputConsole::setTitle(std::string title) {
    if (dialog) {
        dialog->SetTitle(title);
    }

    if (adsb_console) {
        adsb_console->SetTitle(title);
    }

    dialogTitle = title;
}

void ModemDigitalOutputConsole::write(std::string outp) {
    if (dialog) {
        dialog->write(outp);
    }

    if (adsb_console) {
        adsb_decoder::ByteArray frame_bytes;
        int frame_idx = 0;
        for (char c : outp) {
            frame_bytes[frame_idx] = static_cast<uint8_t>(c);

            if (frame_idx == (adsb_decoder::BYTES_PER_FRAME - 1)) {
                adsb_console->push_frame(frame_bytes);
                frame_idx = 0;
            } else {
                frame_idx++;
            }
        }
    }
}

void ModemDigitalOutputConsole::write(char outc) {
    if (!dialog) {
        return;
    }
    dialog->write(outc);
}
