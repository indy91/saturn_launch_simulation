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

#pragma once

#include <wx/wx.h>

#include "SettingsDialog.h"

class SaturnLaunchSimThread;

// Declare the application class
class SaturnLaunchSimApp : public wxApp
{
public:
	SaturnLaunchSimApp();
	virtual ~SaturnLaunchSimApp();

	// Called on application startup
	bool OnInit() override;

	SaturnLaunchSimThread* m_Thread;
};

struct SpacecraftControls
{
	SpacecraftControls() :
		bSIISIVBLVStage(false),
		bTLIInhibit(false)
	{

	}

	bool bSIISIVBLVStage;
	bool bTLIInhibit;
};

// Declare our main frame class
class SaturnLaunchSimFrame : public wxFrame
{
public:
	SaturnLaunchSimFrame(const wxString& title);
	virtual ~SaturnLaunchSimFrame();

	SettingsData m_settingsData;
	SpacecraftControls m_SpacecraftControls;

	wxTextCtrl* EventsOutput;
	wxTextCtrl* PIOOutput;
private:
	void OnOpen(wxCommandEvent& event);
	void OnSave(wxCommandEvent& event);
	void OnButtonRun(wxCommandEvent& event);
	void OnButtonPause(wxCommandEvent& event);
	void OnButtonFastForward(wxCommandEvent& event);
	void OnButton_SII_SIVB_LV_Stage(wxCommandEvent& event);
	void OnButton_TLI_Inhibit(wxCommandEvent& event);
	void OnWorkerEvent(wxThreadEvent& event);
	void OnPropertySheet(wxCommandEvent& event);

	wxTextCtrl* OutputTest;

	// This class handles events
	DECLARE_EVENT_TABLE()
};

class SaturnLaunchSimThread : public wxThread
{
public:
	SaturnLaunchSimThread(SaturnLaunchSimFrame* frame);
	virtual ~SaturnLaunchSimThread();

	// thread execution starts here
	virtual void* Entry() override;

	bool GetPaused();
	void SetPaused(bool set);
	bool GetFastForward();
	void SetFastForward(bool set);

	void SendEventMessage(double t, const std::string& message);
	void SendTelemetryMessage(double t, const std::string& message);
	int GetControl(int num);
private:
	SaturnLaunchSimFrame* m_frame;
	bool paused;
	bool fastforward;
};