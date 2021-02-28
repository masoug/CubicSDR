#include <wx/wx.h>

#include "ADSBFrame.hpp"
#include "ADSBDecoder.hpp"


ADSBFrame::ADSBFrame(wxWindow* parent)
    : wxFrame(parent, wxID_ANY, wxT("ADSB Decoder"), wxDefaultPosition, wxDefaultSize,
              wxCAPTION|wxFRAME_FLOAT_ON_PARENT|wxMAXIMIZE|wxMINIMIZE|wxRESIZE_BORDER|wxFULL_REPAINT_ON_RESIZE),
      m_aircraft_grid(nullptr), m_refresh_timer()
{
    SetExtraStyle(wxWS_EX_PROCESS_UI_UPDATES);

    auto* main_sizer = new wxBoxSizer(wxVERTICAL);

    m_aircraft_grid = new wxGrid(this, wxID_ANY);
//    m_aircraft_grid->DisableDragGridSize();
//    m_aircraft_grid->DisableDragRowSize();
//    m_aircraft_grid->DisableDragColSize();
//    m_aircraft_grid->DisableDragColMove();
    m_aircraft_grid->HideRowLabels();

    // Then we call CreateGrid to set the dimensions of the grid
    m_aircraft_grid->CreateGrid(10, GRID_WIDTH);
    m_aircraft_grid->SetColLabelValue(0, wxT("ICAO"));
    m_aircraft_grid->SetColLabelValue(1, wxT("Callsign"));
    m_aircraft_grid->SetColLabelValue(2, wxT("Altitude"));
    m_aircraft_grid->SetColLabelValue(3, wxT("Latitude"));
    m_aircraft_grid->SetColLabelValue(4, wxT("Longitude"));
    main_sizer->Add(m_aircraft_grid, 1, wxEXPAND | wxALL, 10);

    m_stats_text = new wxStaticText(this, wxID_ANY, wxT("No Received Messages Yet"));
    main_sizer->Add(m_stats_text, 1, wxEXPAND | wxALL, 10);

    SetSizer(main_sizer);
    Layout();

    m_refresh_timer.SetOwner(this, wxID_ANY);
    m_refresh_timer.Start(250);
    Connect(wxID_ANY, wxEVT_TIMER, wxTimerEventHandler(ADSBFrame::DoRefresh));
    Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(ADSBFrame::OnClose));
}

ADSBFrame::~ADSBFrame()
{
    Disconnect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(ADSBFrame::OnClose));
    Disconnect(wxID_ANY, wxEVT_TIMER, wxTimerEventHandler(ADSBFrame::DoRefresh));
}

