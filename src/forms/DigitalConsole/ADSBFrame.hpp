#ifndef CUBICSDR_ADSBFRAME_HPP
#define CUBICSDR_ADSBFRAME_HPP


#include <mutex>
#include <list>
#include <map>

#include <wx/frame.h>
#include <wx/grid.h>
#include <wx/timer.h>
#include <wx/stattext.h>

#include "ADSBDecoder.hpp"


class ADSBFrame : public wxFrame {
public:
    explicit ADSBFrame(wxWindow* parent);
    ~ADSBFrame() override;

protected:

    static constexpr int GRID_WIDTH = 5;

    // Virtual event handlers, overide them in your derived class
    virtual void OnClose( wxCloseEvent& event ) { event.Skip(); }
    virtual void DoRefresh( wxTimerEvent& event ) { event.Skip(); }

    wxGrid* m_aircraft_grid;
    wxTimer m_refresh_timer;
    wxStaticText* m_stats_text;

};


#endif //CUBICSDR_ADSBFRAME_HPP
