/***************************************************************************

Saturn Launch Simulation
Copyright (C) 2026  Niklas Beug

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

***************************************************************************/

#include "SaturnLaunchSimulationGUI.h"
#include "../base/SaturnLaunchSimulation.h"
#include <chrono>
#include "LaunchSimGUIInterface.h"
#include <wx/fileconf.h>
#include <wx/tglbtn.h>
#include "../base/connector_messages.h"

enum
{
	ID_Button_Run = 1,
	ID_Button_Pause,
	ID_Button_FastForward,
	ID_Button_SIISIVB_LV_Stage,
	ID_Button_TLI_Inhibit,
	DIALOGS_PROPERTY_SHEET,
	WORKER_EVENT = wxID_HIGHEST
};

SaturnLaunchSimApp::SaturnLaunchSimApp()
{
	m_Thread = NULL;
}

SaturnLaunchSimApp::~SaturnLaunchSimApp()
{
	if (m_Thread)
	{
		m_Thread->Delete();
	}
}

// This defines the equivalent of main() for the current platform.
wxIMPLEMENT_APP(SaturnLaunchSimApp);

BEGIN_EVENT_TABLE(SaturnLaunchSimFrame, wxFrame)
EVT_BUTTON(ID_Button_Run, SaturnLaunchSimFrame::OnButtonRun)
EVT_BUTTON(ID_Button_Pause, SaturnLaunchSimFrame::OnButtonPause)
EVT_BUTTON(ID_Button_FastForward, SaturnLaunchSimFrame::OnButtonFastForward)
EVT_TOGGLEBUTTON(ID_Button_SIISIVB_LV_Stage, SaturnLaunchSimFrame::OnButton_SII_SIVB_LV_Stage)
EVT_TOGGLEBUTTON(ID_Button_TLI_Inhibit, SaturnLaunchSimFrame::OnButton_TLI_Inhibit)
EVT_THREAD(WORKER_EVENT, SaturnLaunchSimFrame::OnWorkerEvent)
EVT_MENU(DIALOGS_PROPERTY_SHEET, SaturnLaunchSimFrame::OnPropertySheet)
EVT_MENU(wxID_OPEN, SaturnLaunchSimFrame::OnOpen)
EVT_MENU(wxID_SAVE, SaturnLaunchSimFrame::OnSave)
END_EVENT_TABLE()

bool SaturnLaunchSimApp::OnInit()
{
	// Create the main application window
	SaturnLaunchSimFrame* frame = new SaturnLaunchSimFrame("Saturn Launch Simulation");

	wxMenuBar* menuBar = new wxMenuBar;

	wxMenu* fileMenu = new wxMenu;

	fileMenu->Append(wxID_OPEN, "&Open...\tCtrl+O", "");
	fileMenu->Append(wxID_SAVE, "&Save...\tCtrl+S", "");
	fileMenu->AppendSeparator();
	fileMenu->Append(wxID_EXIT, "E&xit\tAlt-X",
		"Quit this program");

	menuBar->Append(fileMenu, "&File");

	wxMenu* settingsMenu = new wxMenu;
	settingsMenu->Append(DIALOGS_PROPERTY_SHEET, "&Settings\tShift-Ctrl-P");

	menuBar->Append(settingsMenu, "&Options");

	frame->SetMenuBar(menuBar);

	// Show it
	frame->Show(true);
	// Start the event loop
	return true;
}

SaturnLaunchSimFrame::SaturnLaunchSimFrame(const wxString& title)
	: wxFrame(NULL, wxID_ANY, title)
{

	// Create a top-level panel to hold all the contents of the frame
	wxPanel* panel = new wxPanel(this, wxID_ANY);

	wxBoxSizer* topSizer = new wxBoxSizer(wxHORIZONTAL);

	wxBoxSizer* inputsSizer = new wxBoxSizer(wxHORIZONTAL);

	wxBoxSizer* buttonsSizer = new wxBoxSizer(wxHORIZONTAL);

	wxButton* button = new wxButton(panel, ID_Button_Run, "Start", wxDefaultPosition);
	wxButton* button2 = new wxButton(panel, ID_Button_Pause, "Pause", wxDefaultPosition);
	wxButton* button3 = new wxButton(panel, ID_Button_FastForward, "Fast Forward", wxDefaultPosition);

	buttonsSizer->Add(button, 0, wxALIGN_CENTER_VERTICAL, 0);
	buttonsSizer->Add(button2, 0, wxALIGN_CENTER_VERTICAL, 0);
	buttonsSizer->Add(button3, 0, wxALIGN_CENTER_VERTICAL, 0);

	inputsSizer->Add(buttonsSizer, 1, wxALIGN_CENTER_VERTICAL | wxALL, 5);

	// Spacecraft inputs
	wxStaticBox* staticBox = new wxStaticBox(panel, wxID_ANY, "Spacecraft Controls");
	wxStaticBoxSizer* staticSizer = new wxStaticBoxSizer(staticBox, wxVERTICAL);

	wxToggleButton* LVStageButton = new wxToggleButton(panel, ID_Button_SIISIVB_LV_Stage, "S-II/S-IVB LV Stage");
	staticSizer->Add(LVStageButton, 0, wxALIGN_CENTER_HORIZONTAL, 0);

	wxToggleButton* TLIInhibitButton = new wxToggleButton(panel, ID_Button_SIISIVB_LV_Stage, "TLI Inhibit");
	staticSizer->Add(TLIInhibitButton, 0, wxALIGN_CENTER_HORIZONTAL, 0);

	inputsSizer->Add(staticSizer, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

	wxBoxSizer* outputSizer = new wxBoxSizer(wxHORIZONTAL);

	// Trajectory
	staticBox = new wxStaticBox(panel, wxID_ANY, "Trajectory");
	staticSizer = new wxStaticBoxSizer(staticBox, wxVERTICAL);

	OutputTest = new wxTextCtrl(panel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(256, 256), wxTE_MULTILINE | wxTE_READONLY);
	staticSizer->Add(OutputTest, 0, wxALIGN_LEFT | wxALL, 5);

	outputSizer->Add(staticSizer, 0, wxALIGN_CENTER_VERTICAL, 10);

	// Events
	staticBox = new wxStaticBox(panel, wxID_ANY, "Events");
	staticSizer = new wxStaticBoxSizer(staticBox, wxVERTICAL);

	EventsOutput = new wxTextCtrl(panel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(386, 256), wxTE_MULTILINE | wxTE_READONLY);
	staticSizer->Add(EventsOutput, 0, wxALIGN_LEFT | wxALL, 5);

	outputSizer->Add(staticSizer, 0, wxALIGN_CENTER_VERTICAL, 10);

	// Launch Computer
	staticBox = new wxStaticBox(panel, wxID_ANY, "Launch Computer");
	staticSizer = new wxStaticBoxSizer(staticBox, wxVERTICAL);

	PIOOutput = new wxTextCtrl(panel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(256, 256), wxTE_MULTILINE | wxTE_READONLY);
	staticSizer->Add(PIOOutput, 0, wxALIGN_LEFT | wxALL, 5);

	outputSizer->Add(staticSizer, 0, wxALIGN_CENTER_VERTICAL, 10);

	// Set up the sizer for the panel
	wxBoxSizer* panelSizer = new wxBoxSizer(wxVERTICAL);

	panelSizer->Add(inputsSizer, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 10);
	panelSizer->Add(outputSizer, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 10);
	panel->SetSizer(panelSizer);

	topSizer->Add(panel, 1, wxEXPAND);

	SetSizer(topSizer); // use the sizer for layout
	topSizer->Fit(this); // fit the dialog to the contents
	topSizer->SetSizeHints(this); // set hints to honor min size
};

SaturnLaunchSimFrame::~SaturnLaunchSimFrame()
{
	
}

void SaturnLaunchSimFrame::OnOpen(wxCommandEvent& event)
{
	wxFileName fn(wxGetCwd());
	wxString defaultDir = fn.GetPath();

	wxFileDialog dlg(this, "Open file", defaultDir, wxEmptyString, "Ini files (*.ini)|*.ini", wxFD_OPEN | wxFD_FILE_MUST_EXIST);

	if (dlg.ShowModal() == wxID_CANCEL)
	{
		return;
	}

	wxFileConfig* config = new wxFileConfig("Saturn Launch Sim", wxEmptyString, dlg.GetPath(), wxEmptyString, wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

	config->Read("General/LaunchTime", &m_settingsData.m_LaunchTime);
	config->Read("General/CountdownTime", &m_settingsData.m_CountdownTime);
	config->Read("General/LVDCSoftwareName", &m_settingsData.m_LVDCSoftwareName);
	config->Read("Launchpad/LaunchpadLatitude", &m_settingsData.m_LaunchpadLatitude);
	config->Read("Launchpad/LaunchpadLongitude", &m_settingsData.m_LaunchpadLongitude);
	config->Read("Launchpad/LaunchpadAltitude", &m_settingsData.m_LaunchpadAltitude);
	config->Read("Launchpad/LaunchpadAzimuth", &m_settingsData.m_LaunchpadAzimuth);
	config->Read("Masses/SI_EmptyMass", &m_settingsData.m_SI_EmptyMass);
	config->Read("Masses/SI_MainFuelMass", &m_settingsData.m_SI_MainFuelMass);
	config->Read("Masses/SI_RetroFuelMass", &m_settingsData.m_SI_RetroFuelMass);
	config->Read("Masses/SI_RetroNum", &m_settingsData.m_SI_RetroNum);
	config->Read("Masses/SII_EmptyMass", &m_settingsData.m_SII_EmptyMass);
	config->Read("Masses/SII_MainFuelMass", &m_settingsData.m_SII_MainFuelMass);
	config->Read("Masses/SII_InterstageMass", &m_settingsData.m_SII_InterstageMass);
	config->Read("Masses/SII_UllageFuelMass", &m_settingsData.m_SII_UllageFuelMass);
	config->Read("Masses/SII_UllageNum", &m_settingsData.m_SII_UllageNum);
	config->Read("Masses/SII_RetroFuelMass", &m_settingsData.m_SII_RetroFuelMass);
	config->Read("Masses/SII_RetroNum", &m_settingsData.m_SII_RetroNum);
	config->Read("Masses/S4B_EmptyMass", &m_settingsData.m_S4B_EmptyMass);
	config->Read("Masses/S4PL_Mass", &m_settingsData.m_S4PL_Mass);
	config->Read("Masses/S4B_MainFuelMass", &m_settingsData.m_S4B_MainFuelMass);
	config->Read("Masses/S4B_UllageCaseEmptyMass", &m_settingsData.m_S4B_UllageCaseEmptyMass);
	config->Read("Masses/S4B_UllageFuelMass", &m_settingsData.m_S4B_UllageFuelMass);
	config->Read("Masses/S4B_UllageNum", &m_settingsData.m_S4B_UllageNum);
	config->Read("Masses/S4B_APSPropellant", &m_settingsData.m_S4B_APSPropellant);
	config->Read("Masses/LET_Mass", &m_settingsData.m_LET_Mass);

	delete config;
}

void SaturnLaunchSimFrame::OnSave(wxCommandEvent& event)
{
	wxFileName fn(wxGetCwd());
	wxString defaultDir = fn.GetPath();

	wxFileDialog dlg(this, "Save file", defaultDir, wxEmptyString, "Ini files (*.ini)|*.ini", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

	if (dlg.ShowModal() == wxID_CANCEL)
	{
		return;
	}

	wxFileConfig* config = new wxFileConfig("Saturn Launch Sim", wxEmptyString, dlg.GetPath(), wxEmptyString, wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

	config->Write("General/LaunchTime", m_settingsData.m_LaunchTime);
	config->Write("General/CountdownTime", m_settingsData.m_CountdownTime);
	config->Write("General/LVDCSoftwareName", m_settingsData.m_LVDCSoftwareName);
	config->Write("Launchpad/LaunchpadLatitude", m_settingsData.m_LaunchpadLatitude);
	config->Write("Launchpad/LaunchpadLongitude", m_settingsData.m_LaunchpadLongitude);
	config->Write("Launchpad/LaunchpadAltitude", m_settingsData.m_LaunchpadAltitude);
	config->Write("Launchpad/LaunchpadAzimuth", m_settingsData.m_LaunchpadAzimuth);
	config->Write("Masses/SI_EmptyMass", m_settingsData.m_SI_EmptyMass);
	config->Write("Masses/SI_MainFuelMass", m_settingsData.m_SI_MainFuelMass);
	config->Write("Masses/SI_RetroFuelMass", m_settingsData.m_SI_RetroFuelMass);
	config->Write("Masses/SI_RetroNum", m_settingsData.m_SI_RetroNum);
	config->Write("Masses/SII_EmptyMass", m_settingsData.m_SII_EmptyMass);
	config->Write("Masses/SII_MainFuelMass", m_settingsData.m_SII_MainFuelMass);
	config->Write("Masses/SII_InterstageMass", m_settingsData.m_SII_InterstageMass);
	config->Write("Masses/SII_UllageFuelMass", m_settingsData.m_SII_UllageFuelMass);
	config->Write("Masses/SII_UllageNum", m_settingsData.m_SII_UllageNum);
	config->Write("Masses/SII_RetroFuelMass", m_settingsData.m_SII_RetroFuelMass);
	config->Write("Masses/SII_RetroNum", m_settingsData.m_SII_RetroNum);
	config->Write("Masses/S4B_EmptyMass", m_settingsData.m_S4B_EmptyMass);
	config->Write("Masses/S4PL_Mass", m_settingsData.m_S4PL_Mass);
	config->Write("Masses/S4B_MainFuelMass", m_settingsData.m_S4B_MainFuelMass);
	config->Write("Masses/S4B_UllageCaseEmptyMass", m_settingsData.m_S4B_UllageCaseEmptyMass);
	config->Write("Masses/S4B_UllageFuelMass", m_settingsData.m_S4B_UllageFuelMass);
	config->Write("Masses/S4B_UllageNum", m_settingsData.m_S4B_UllageNum);
	config->Write("Masses/S4B_APSPropellant", m_settingsData.m_S4B_APSPropellant);
	config->Write("Masses/LET_Mass", m_settingsData.m_LET_Mass);

	delete config;

	SetTitle(dlg.GetPath() + " - Saturn Launch Simulation");
}

void SaturnLaunchSimFrame::OnButtonRun(wxCommandEvent& event)
{
	if (wxGetApp().m_Thread)
	{
		return;
		//wxGetApp().m_Thread->Delete();
		//wxGetApp().m_Thread = NULL;
	}

	OutputTest->Clear();
	EventsOutput->Clear();
	PIOOutput->Clear();

	SaturnLaunchSimThread* thread = new SaturnLaunchSimThread(this);

	wxGetApp().m_Thread = thread;

	thread->Run();
}

void SaturnLaunchSimFrame::OnButtonPause(wxCommandEvent& event)
{
	if (wxGetApp().m_Thread)
	{
		if (wxGetApp().m_Thread->GetPaused())
		{
			wxGetApp().m_Thread->SetPaused(false);
		}
		else
		{
			wxGetApp().m_Thread->SetPaused(true);
		}
	}
}

void SaturnLaunchSimFrame::OnButtonFastForward(wxCommandEvent& event)
{
	if (wxGetApp().m_Thread)
	{
		if (wxGetApp().m_Thread->GetFastForward())
		{
			wxGetApp().m_Thread->SetFastForward(false);
		}
		else
		{
			wxGetApp().m_Thread->SetFastForward(true);
		}
	}
}

void SaturnLaunchSimFrame::OnButton_SII_SIVB_LV_Stage(wxCommandEvent& event)
{
	m_SpacecraftControls.bSIISIVBLVStage = event.IsChecked();
}

void SaturnLaunchSimFrame::OnButton_TLI_Inhibit(wxCommandEvent& event)
{
	m_SpacecraftControls.bTLIInhibit = event.IsChecked();
}

void SaturnLaunchSimFrame::OnWorkerEvent(wxThreadEvent& event)
{
	SaturnLaunchSimulationOutputs out = event.GetPayload<SaturnLaunchSimulationOutputs>();

	OutputTest->Clear();
	OutputTest->AppendText(wxString::Format("T%+.1lf", out.traj.T));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("V %.2lf m/s", out.traj.V));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Alt %.2lf km", out.traj.Alt));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Hdot %.1lf m/s", out.traj.Hdot));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("F %.1lf N", out.traj.F));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Mass %.1lf kg", out.traj.m));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Attitude %06.2lf %06.2lf %06.2lf", out.traj.Roll, out.traj.Pitch, out.traj.Yaw));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Rates %.4lf %.4lf %.4lf", out.traj.Rates[0], out.traj.Rates[1], out.traj.Rates[2]));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Q %.1lf N/m^2", out.traj.Q));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Aerodynamic Force %.1lf N", out.traj.F_aero));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Angle of Attack %.2lf deg", out.traj.alpha));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Latitude %.3lf deg", out.traj.Latitude));
	OutputTest->AppendText("\n");
	OutputTest->AppendText(wxString::Format("Longitude %.3lf deg", out.traj.Longitude));

	PIOOutput->Clear();
	PIOOutput->AppendText("Program: " + out.rca.TestProgram);
	PIOOutput->AppendText("\n");
	PIOOutput->AppendText(wxString::Format("Step: %d", out.rca.State));
	PIOOutput->AppendText("\n");
	PIOOutput->AppendText(wxString::Format("SubStep: %d", out.rca.SubState));
	PIOOutput->AppendText("\n");
	PIOOutput->AppendText(wxString::Format("Mode Register: %o", out.rca.ModeRegister));
	PIOOutput->AppendText("\n");
	PIOOutput->AppendText(wxString::Format("Status Code: %o", out.rca.StatusCode));
	PIOOutput->AppendText("\n");
	PIOOutput->AppendText(wxString::Format("Last Tag: %o %o", out.rca.LastPIO, out.rca.LastPIOValue));
	PIOOutput->AppendText("\n");
	PIOOutput->AppendText(wxString::Format("LVDC Error Message: %o", out.rca.LVDCErrorMessage));
}

void SaturnLaunchSimFrame::OnPropertySheet(wxCommandEvent& event)
{
	SettingsDialog dialog(this, m_settingsData);
	dialog.ShowModal();
}

SaturnLaunchSimThread::SaturnLaunchSimThread(SaturnLaunchSimFrame* frame): wxThread()
{
	m_frame = frame;
	paused = false;
	fastforward = false;
}

SaturnLaunchSimThread::~SaturnLaunchSimThread()
{

}

void SettingsToInputs(const SettingsData& set, SaturnLaunchSimulationInputs& in)
{
	in.t_Launch = set.m_LaunchTime;
	in.t_Countdown = set.m_CountdownTime;
	in.IsSaturnV = (set.m_IsSaturnV == 0);
	in.LVDCSoftwareName = set.m_LVDCSoftwareName;

	in.lat_geod = set.m_LaunchpadLatitude;
	in.lng = set.m_LaunchpadLongitude;
	in.alt = set.m_LaunchpadAltitude;
	in.A_ZP = set.m_LaunchpadAzimuth;

	in.SI_EmptyMass = set.m_SI_EmptyMass;
	in.SI_MainFuelMass = set.m_SI_MainFuelMass;
	in.SI_RetroFuelMass = set.m_SI_RetroFuelMass;
	in.SI_RetroNum = set.m_SI_RetroNum;

	in.SII_EmptyMass = set.m_SII_EmptyMass;
	in.SII_MainFuelMass = set.m_SII_MainFuelMass;
	in.SII_InterstageMass = set.m_SII_InterstageMass;
	in.SII_UllageFuelMass = set.m_SII_UllageFuelMass;
	in.SII_UllageNum = set.m_SII_UllageNum;
	in.SII_RetroFuelMass = set.m_SII_RetroFuelMass;
	in.SII_RetroNum = set.m_SII_RetroNum;

	in.S4B_EmptyMass = set.m_S4B_EmptyMass;
	in.S4PL_Mass = set.m_S4PL_Mass;
	in.S4B_MainFuelMass = set.m_S4B_MainFuelMass;
	in.S4B_UllageFuelMass = set.m_S4B_UllageFuelMass;
	in.S4B_UllageNum = set.m_S4B_UllageNum;
	in.S4B_APSPropellant = set.m_S4B_APSPropellant;
	in.LET_Mass = set.m_LET_Mass;
}

void* SaturnLaunchSimThread::Entry()
{
	OrbMech::GlobalConstants cnst;

	SaturnLaunchSimulation sim(cnst);

	SaturnLaunchSimulationInputs in;

	LaunchSimGUIInterface interf(this);

	SettingsToInputs(m_frame->m_settingsData, in);

	in.gui_interface = &interf;

	if (sim.Init(in))
	{
		// Error
		wxGetApp().m_Thread = NULL;
		return nullptr;
	}

	double dt_step = (1.0 / 50.0);

	int counter = 0;

	int num, i;

	while (true)
	{
		// check if just this thread was asked to exit
		if (TestDestroy())
			break;
		if (paused)
		{
			wxThread::Sleep(10);
			continue;
		}

		if (fastforward)
		{
			num = 10;
		}
		else
		{
			num = 1;
		}

		for (i = 0; i < num; i++)
		{
			sim.Step(dt_step);
		}

		counter++;

		if (counter > 20)
		{
			wxThreadEvent event(wxEVT_THREAD, WORKER_EVENT);

			event.SetPayload(sim.out);

			wxQueueEvent(m_frame, event.Clone());

			counter = 0;
		}

		wxThread::Sleep(5);
	}

	return nullptr;
}

bool SaturnLaunchSimThread::GetPaused()
{
	return paused;
}

void SaturnLaunchSimThread::SetPaused(bool set)
{
	paused = set;
}

bool SaturnLaunchSimThread::GetFastForward()
{
	return fastforward;
}

void SaturnLaunchSimThread::SetFastForward(bool set)
{
	fastforward = set;
}

void SaturnLaunchSimThread::SendEventMessage(double t, const std::string& message)
{
	std::string mess = std::to_string(t) + ": " + message + "\n";

	m_frame->EventsOutput->AppendText(mess);

	//wxMessageOutputBest out;

	//out.Output(mess);
}

void SaturnLaunchSimThread::SendTelemetryMessage(double t, const std::string& message)
{
	std::string mess = std::to_string(t) + ": " + message + "\n";

	m_frame->PIOOutput->AppendText(mess);
}

int SaturnLaunchSimThread::GetControl(int num)
{
	switch (num)
	{
	case IUCSM_GET_SII_SIVB_LV_STAGE:
		return m_frame->m_SpacecraftControls.bSIISIVBLVStage ? 1 : 0;
	case IUCSM_GET_TLI_INHIBIT:
		return m_frame->m_SpacecraftControls.bTLIInhibit ? 1 : 0;
	case IUCSM_GET_LV_TAKEOVER:
		return 0; // TBD
	case IUCSM_GET_SIVB_INJ_SEQ_START:
		if (false && false) // TBD
		{
			return 1;
		}
		return 0;
	case IUCSM_GET_SIVB_CUTOFF:
		if (false && false) // TBD
		{
			return 1;
		}
		return 0;
	}
	return 0;
}