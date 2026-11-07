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

#include "SettingsDialog.h"
#include <wx/bookctrl.h>
#include <wx/valgen.h>
#include <wx/tokenzr.h>

wxIMPLEMENT_CLASS(SettingsDialog, wxPropertySheetDialog);

wxBEGIN_EVENT_TABLE(SettingsDialog, wxPropertySheetDialog)
EVT_BUTTON(wxID_OK, SettingsDialog::OnOK)
EVT_BUTTON(wxID_APPLY, SettingsDialog::OnApply)
EVT_BUTTON(wxID_CANCEL, SettingsDialog::OnCancel)
EVT_TEXT(ID_TEXT_CTRL, SettingsDialog::OnTextChange)
EVT_CHOICE(ID_CHOICE, SettingsDialog::OnChoiceChange)
wxEND_EVENT_TABLE()

SettingsDialog::SettingsDialog(wxWindow* win, SettingsData& settingsData) :
m_settingsData(settingsData)
{
	int resizeBorder = wxRESIZE_BORDER;

	Create(win, wxID_ANY, "Settings", wxDefaultPosition, wxDefaultSize,
		wxDEFAULT_DIALOG_STYLE | resizeBorder);

	CreateButtons(wxOK | wxAPPLY | wxCANCEL);

	wxBookCtrlBase* notebook = GetBookCtrl();

	wxPanel* generalSettings = CreateGeneralSettingsPage(notebook);
	wxPanel* launchpadSettings = CreateLaunchpadSettingsPage(notebook);
	wxPanel* massSettings = CreateMassesSettingsPage(notebook);
	wxPanel* propSettings = CreatePropulsionSettingsPage(notebook);

	notebook->AddPage(generalSettings, "General", true);
	notebook->AddPage(launchpadSettings, "Launchpad", false);
	notebook->AddPage(massSettings, "Masses", false);
	notebook->AddPage(propSettings, "Propulsion", false);

	LoadData();
	EnableUpdate(false);

	LayoutDialog();

	mDataChanged = false;
}

SettingsDialog::~SettingsDialog()
{

}

wxPanel* SettingsDialog::CreateGeneralSettingsPage(wxWindow* parent)
{
	wxPanel* panel = new wxPanel(parent, wxID_ANY);

	wxBoxSizer* topSizer = new wxBoxSizer(wxVERTICAL);

	wxFlexGridSizer* flexGridSizer = new wxFlexGridSizer(0, 3, 0, 0);

	wxArrayString strings;

	// Initial time
	textLaunchTimeGMT = CreateTextCtrl(panel, flexGridSizer, "Launch Time", "hh:mm:ss", "GMT launch time");
	textCountdownTime = CreateTextCtrl(panel, flexGridSizer, "Countdown Time", "sec", "Initial countdown time (positive value) in seconds");

	strings.Add("Saturn V");
	strings.Add("Saturn IB");
	choiceSaturnType = CreateChoice(panel, flexGridSizer, "Type", "Bla", strings);
	strings.clear();

	textLVDCSoftwareName = CreateTextCtrl(panel, flexGridSizer, "LVDC Software", "", "Name of the LVDC software (without file extension, located in the files folder)");

	topSizer->Add(flexGridSizer, wxSizerFlags(1).Expand().Border());



	panel->SetSizerAndFit(topSizer);

	return panel;
}

wxPanel* SettingsDialog::CreateLaunchpadSettingsPage(wxWindow* parent)
{
	wxPanel* panel = new wxPanel(parent, wxID_ANY);

	wxBoxSizer* topSizer = new wxBoxSizer(wxVERTICAL);

	wxFlexGridSizer* flexGridSizer = new wxFlexGridSizer(0, 3, 0, 0);

	textLaunchpadLatitude = CreateTextCtrl(panel, flexGridSizer, "Latitude", "deg", "Geodetic launchpad latitude in degrees");
	textLaunchpadLongitude = CreateTextCtrl(panel, flexGridSizer, "Longitude", "deg", "Launchpad longitude in degrees");
	textLaunchpadAltitude = CreateTextCtrl(panel, flexGridSizer, "Altitude", "m", "Launchpad altitude above reference ellipsoid in meters");
	textLaunchpadAzimuth = CreateTextCtrl(panel, flexGridSizer, "Azimuth", "deg", "Launchpad azimuth in degrees");

	topSizer->Add(flexGridSizer, wxSizerFlags(1).Expand().Border());

	panel->SetSizerAndFit(topSizer);

	return panel;
}

wxPanel* SettingsDialog::CreateMassesSettingsPage(wxWindow* parent)
{
	wxPanel* panel = new wxPanel(parent, wxID_ANY);

	wxBoxSizer* topSizer = new wxBoxSizer(wxVERTICAL);
	wxBoxSizer* midSizer = new wxBoxSizer(wxHORIZONTAL);

	wxFlexGridSizer* flexGridSizer1 = new wxFlexGridSizer(0, 3, 0, 0);

	textSI_EmptyMass = CreateTextCtrl(panel, flexGridSizer1, "S-I Empty Mass", "kg", "S-IB or S-IC empty mass in kilograms");
	textSI_MainFuelMass = CreateTextCtrl(panel, flexGridSizer1, "S-I Propellant Mass", "kg", "S-IB or S-IC propellant mass in kilograms");
	textSI_RetroFuelMass = CreateTextCtrl(panel, flexGridSizer1, "S-I Retro Mass", "kg", "S-IB or S-IC retro propellant mass per rocket in kilograms");
	textSI_RetroNum = CreateTextCtrl(panel, flexGridSizer1, "S-I Retro Num", "", "Number of S-IB or S-IC retro rockets");
	textSII_EmptyMass = CreateTextCtrl(panel, flexGridSizer1, "S-II Empty Mass", "kg", "S-II empty mass in kilograms");
	textSII_MainFuelMass = CreateTextCtrl(panel, flexGridSizer1, "S-II Propellant Mass", "kg", "S-II propellant mass in kilograms");
	textSII_InterstageMass = CreateTextCtrl(panel, flexGridSizer1, "S-II Interstage Mass", "kg", "S-II interstage mass in kilograms");
	textSII_UllageFuelMass = CreateTextCtrl(panel, flexGridSizer1, "S-II Ullage Mass", "kg", "S-II ullage propellant mass per rocket in kilograms");
	textSII_UllageNum = CreateTextCtrl(panel, flexGridSizer1, "S-II Ullage Num", "", "Number of S-II ullage rockets");
	textSII_RetroFuelMass = CreateTextCtrl(panel, flexGridSizer1, "S-II Retro Mass", "kg", "S-II retro propellant mass per rocket in kilograms");
	textSII_RetroNum = CreateTextCtrl(panel, flexGridSizer1, "S-II Retro Num", "", "Number of S-II retro rockets");

	wxFlexGridSizer* flexGridSizer2 = new wxFlexGridSizer(0, 3, 0, 0);

	textS4B_EmptyMass = CreateTextCtrl(panel, flexGridSizer2, "S-IVB Empty Mass", "kg", "S-IVB empty mass (including IU and SLA) in kilograms");
	textS4PL_Mass = CreateTextCtrl(panel, flexGridSizer2, "Payload Mass", "kg", "Payload mass (e.g. CSM and LM) in kilograms");
	textS4B_MainFuelMass = CreateTextCtrl(panel, flexGridSizer2, "S-IVB Propellant Mass", "kg", "S-IVB propellant mass in kilograms");
	textS4B_UllageFuelMass = CreateTextCtrl(panel, flexGridSizer2, "S-IVB Ullage Case Mass", "kg", "S-IVB ullage case mass per rocket in kilograms");
	textS4B_UllageCaseEmptyMass = CreateTextCtrl(panel, flexGridSizer2, "S-IVB Ullage Mass", "kg", "S-IVB ullage propellant mass per rocket in kilograms");
	textS4B_UllageNum = CreateTextCtrl(panel, flexGridSizer2, "S-IVB Ullage Num", "", "Number of S-IVB ullage rockets");
	textS4B_APSPropellant = CreateTextCtrl(panel, flexGridSizer2, "APS Propellant Mass", "kg", "S-IVB APS propellant mass per module in kilograms");
	textLET_Mass = CreateTextCtrl(panel, flexGridSizer2, "LET Mass", "kg", "Launch Escape Tower mass in kilograms");

	midSizer->Add(flexGridSizer1, wxSizerFlags(1).Expand().Border());
	midSizer->Add(flexGridSizer2, wxSizerFlags(1).Expand().Border());

	topSizer->Add(midSizer, wxSizerFlags(1).Expand().Border());

	panel->SetSizerAndFit(topSizer);

	return panel;
}

wxPanel* SettingsDialog::CreatePropulsionSettingsPage(wxWindow* parent)
{
	wxPanel* panel = new wxPanel(parent, wxID_ANY);

	wxBoxSizer* topSizer = new wxBoxSizer(wxVERTICAL);

	panel->SetSizerAndFit(topSizer);

	return panel;
}

void SettingsDialog::LoadData()
{
	wxTimeSpan timeSpan = wxTimeSpan::Seconds(static_cast<long>(m_settingsData.m_LaunchTime));
	wxString formattedTime = timeSpan.Format("%H:%M:%S");
	textLaunchTimeGMT->SetValue(formattedTime);

	textCountdownTime->SetValue(wxString::Format("%g", m_settingsData.m_CountdownTime));
	choiceSaturnType->SetSelection(m_settingsData.m_IsSaturnV);
	textLVDCSoftwareName->SetValue(m_settingsData.m_LVDCSoftwareName);
	textLaunchpadLatitude->SetValue(wxString::Format("%g", m_settingsData.m_LaunchpadLatitude));
	textLaunchpadLongitude->SetValue(wxString::Format("%g", m_settingsData.m_LaunchpadLongitude));
	textLaunchpadAltitude->SetValue(wxString::Format("%g", m_settingsData.m_LaunchpadAltitude));
	textLaunchpadAzimuth->SetValue(wxString::Format("%g", m_settingsData.m_LaunchpadAzimuth));

	textSI_EmptyMass->SetValue(wxString::Format("%g", m_settingsData.m_SI_EmptyMass));
	textSI_MainFuelMass->SetValue(wxString::Format("%g", m_settingsData.m_SI_MainFuelMass));
	textSI_RetroFuelMass->SetValue(wxString::Format("%g", m_settingsData.m_SI_RetroFuelMass));
	textSI_RetroNum->SetValue(wxString::Format("%d", m_settingsData.m_SI_RetroNum));
	textSII_EmptyMass->SetValue(wxString::Format("%g", m_settingsData.m_SII_EmptyMass));
	textSII_MainFuelMass->SetValue(wxString::Format("%g", m_settingsData.m_SII_MainFuelMass));
	textSII_InterstageMass->SetValue(wxString::Format("%g", m_settingsData.m_SII_InterstageMass));
	textSII_UllageFuelMass->SetValue(wxString::Format("%g", m_settingsData.m_SII_UllageFuelMass));
	textSII_UllageNum->SetValue(wxString::Format("%d", m_settingsData.m_SII_UllageNum));
	textSII_RetroFuelMass->SetValue(wxString::Format("%g", m_settingsData.m_SII_RetroFuelMass));
	textSII_RetroNum->SetValue(wxString::Format("%d", m_settingsData.m_SII_RetroNum));
	textS4B_EmptyMass->SetValue(wxString::Format("%g", m_settingsData.m_S4B_EmptyMass));
	textS4PL_Mass->SetValue(wxString::Format("%g", m_settingsData.m_S4PL_Mass));
	textS4B_MainFuelMass->SetValue(wxString::Format("%g", m_settingsData.m_S4B_MainFuelMass));
	textS4B_UllageFuelMass->SetValue(wxString::Format("%g", m_settingsData.m_S4B_UllageFuelMass));
	textS4B_UllageCaseEmptyMass->SetValue(wxString::Format("%g", m_settingsData.m_S4B_UllageCaseEmptyMass));
	textS4B_UllageNum->SetValue(wxString::Format("%d", m_settingsData.m_S4B_UllageNum));
	textS4B_APSPropellant->SetValue(wxString::Format("%g", m_settingsData.m_S4B_APSPropellant));
	textLET_Mass->SetValue(wxString::Format("%g", m_settingsData.m_LET_Mass));
}

void SettingsDialog::SaveData()
{
	wxStringTokenizer tokenizer(textLaunchTimeGMT->GetValue(), ":");
	long hours = 0, minutes = 0, seconds = 0;
	bool success = true;

	if (tokenizer.HasMoreTokens()) {
		if (!tokenizer.GetNextToken().ToLong(&hours)) success = false;
	}
	if (tokenizer.HasMoreTokens()) {
		if (!tokenizer.GetNextToken().ToLong(&minutes)) success = false;
	}
	if (tokenizer.HasMoreTokens()) {
		if (!tokenizer.GetNextToken().ToLong(&seconds)) success = false;
	}

	if (success) {
		wxTimeSpan timeSpan(hours, minutes, seconds, 0);
		m_settingsData.m_LaunchTime = timeSpan.GetMilliseconds().ToDouble() / 1000.0;
	}
	else
	{
		// Error handling
	}

	textCountdownTime->GetValue().ToDouble(&m_settingsData.m_CountdownTime);
	m_settingsData.m_IsSaturnV = choiceSaturnType->GetSelection();
	m_settingsData.m_LVDCSoftwareName = textLVDCSoftwareName->GetValue();
	textLaunchpadLatitude->GetValue().ToDouble(&m_settingsData.m_LaunchpadLatitude);
	textLaunchpadLongitude->GetValue().ToDouble(&m_settingsData.m_LaunchpadLongitude);
	textLaunchpadAltitude->GetValue().ToDouble(&m_settingsData.m_LaunchpadAltitude);
	textLaunchpadAzimuth->GetValue().ToDouble(&m_settingsData.m_LaunchpadAzimuth);

	textSI_EmptyMass->GetValue().ToDouble(&m_settingsData.m_SI_EmptyMass);
	textSI_MainFuelMass->GetValue().ToDouble(&m_settingsData.m_SI_MainFuelMass);
	textSI_RetroFuelMass->GetValue().ToDouble(&m_settingsData.m_SI_RetroFuelMass);
	textSI_RetroNum->GetValue().ToInt(&m_settingsData.m_SI_RetroNum);
	textSII_EmptyMass->GetValue().ToDouble(&m_settingsData.m_SII_EmptyMass);
	textSII_MainFuelMass->GetValue().ToDouble(&m_settingsData.m_SII_MainFuelMass);
	textSII_InterstageMass->GetValue().ToDouble(&m_settingsData.m_SII_InterstageMass);
	textSII_UllageFuelMass->GetValue().ToDouble(&m_settingsData.m_SII_UllageFuelMass);
	textSII_UllageNum->GetValue().ToInt(&m_settingsData.m_SII_UllageNum);
	textSII_RetroFuelMass->GetValue().ToDouble(&m_settingsData.m_SII_RetroFuelMass);
	textSII_RetroNum->GetValue().ToInt(&m_settingsData.m_SII_RetroNum);
	textS4B_EmptyMass->GetValue().ToDouble(&m_settingsData.m_S4B_EmptyMass);
	textS4PL_Mass->GetValue().ToDouble(&m_settingsData.m_S4PL_Mass);
	textS4B_MainFuelMass->GetValue().ToDouble(&m_settingsData.m_S4B_MainFuelMass);
	textS4B_UllageFuelMass->GetValue().ToDouble(&m_settingsData.m_S4B_UllageFuelMass);
	textS4B_UllageCaseEmptyMass->GetValue().ToDouble(&m_settingsData.m_S4B_UllageCaseEmptyMass);
	textS4B_UllageNum->GetValue().ToInt(&m_settingsData.m_S4B_UllageNum);
	textS4B_APSPropellant->GetValue().ToDouble(&m_settingsData.m_S4B_APSPropellant);
	textLET_Mass->GetValue().ToDouble(&m_settingsData.m_LET_Mass);
}

void SettingsDialog::EnableUpdate(bool enable)
{
	if (enable)
	{
		mDataChanged = true;
	}
	else
	{
		mDataChanged = false;
	}
	// toggle the Apply button
	FindWindow(wxID_APPLY)->Enable(mDataChanged);
}

void SettingsDialog::OnApply(wxCommandEvent& event)
{
	if (mDataChanged)
	{
		SaveData();
		EnableUpdate(false);
	}
}

void SettingsDialog::OnOK(wxCommandEvent& event)
{
	if (mDataChanged)
	{
		SaveData();
	}
	EndModal(wxID_OK);
}

void SettingsDialog::OnCancel(wxCommandEvent& event)
{
	EndModal(wxID_CANCEL);
}

void SettingsDialog::OnTextChange(wxCommandEvent& event)
{
	EnableUpdate(true);
}

void SettingsDialog::OnChoiceChange(wxCommandEvent& event)
{
	EnableUpdate(true);
}

wxTextCtrl* SettingsDialog::CreateTextCtrl(wxPanel* panel, wxFlexGridSizer* flexGridSizer, const wxString& label, const wxString& unit, const wxString& tooltip) const
{
	flexGridSizer->Add(new wxStaticText(panel, wxID_ANY, label), wxSizerFlags().CenterVertical().Border());

	wxTextCtrl* item1 = new wxTextCtrl(panel, ID_TEXT_CTRL);
	item1->SetToolTip(tooltip);
	flexGridSizer->Add(item1, wxSizerFlags().CenterVertical().Border());

	flexGridSizer->Add(new wxStaticText(panel, wxID_ANY, unit), wxSizerFlags().CenterVertical().Border());

	return item1;
}

wxChoice* SettingsDialog::CreateChoice(wxPanel* panel, wxFlexGridSizer* flexGridSizer, const wxString& label, const wxString& tooltip, const wxArrayString& strings) const
{
	flexGridSizer->Add(new wxStaticText(panel, wxID_ANY, label), wxSizerFlags().CenterVertical().Border());

	wxChoice* item1 = new wxChoice(panel, ID_CHOICE, wxDefaultPosition, wxDefaultSize, strings);
	item1->SetToolTip(tooltip);
	flexGridSizer->Add(item1, wxSizerFlags().CenterVertical().Border());

	flexGridSizer->Add(new wxStaticText(panel, wxID_ANY, ""), wxSizerFlags().CenterVertical().Border());

	return item1;
}