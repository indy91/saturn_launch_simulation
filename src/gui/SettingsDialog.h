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
#include <wx/propdlg.h>

struct SettingsData
{
    SettingsData() :
        m_LaunchTime(10380.0),
        m_CountdownTime(620.0),
        m_IsSaturnV(0),
        m_LaunchpadLatitude(28.608419),
        m_LaunchpadLongitude(-80.604139),
        m_LaunchpadAltitude(60.5),
        m_LaunchpadAzimuth(90.0),
        m_SI_EmptyMass(133602.0),
        m_SI_MainFuelMass(2146040.0),
        m_SI_RetroFuelMass(126.1),
        m_SI_RetroNum(8),
        m_SII_EmptyMass(39869.0),
        m_SII_MainFuelMass(443500.0),
        m_SII_InterstageMass(3982.0),
        m_SII_UllageFuelMass(152.407),
        m_SII_UllageNum(4),
        m_SII_RetroFuelMass(121.65),
        m_SII_RetroNum(4),
        m_S4B_EmptyMass(13439.0),
        m_S4PL_Mass(49794.0),
        m_S4B_MainFuelMass(107428.0),
        m_S4B_UllageCaseEmptyMass(62.0),
        m_S4B_UllageFuelMass(26.67123),
        m_S4B_UllageNum(2),
        m_S4B_APSPropellant(143.0),
        m_LET_Mass(4051.0),
        m_LVDCSoftwareName("yaASM")
    {
    }

    double m_LaunchTime;
    double m_CountdownTime;
    int m_IsSaturnV;
    double m_LaunchpadLatitude;
    double m_LaunchpadLongitude;
    double m_LaunchpadAltitude;
    double m_LaunchpadAzimuth;

    double m_SI_EmptyMass;
    double m_SI_MainFuelMass;
    double m_SI_RetroFuelMass;
    int m_SI_RetroNum;

    double m_SII_EmptyMass;
    double m_SII_MainFuelMass;
    double m_SII_InterstageMass;
    double m_SII_UllageFuelMass;
    int m_SII_UllageNum;
    double m_SII_RetroFuelMass;
    int m_SII_RetroNum;

    double m_S4B_EmptyMass;
    double m_S4PL_Mass;
    double m_S4B_MainFuelMass;
    double m_S4B_UllageFuelMass;
    double m_S4B_UllageCaseEmptyMass;
    int m_S4B_UllageNum;
    double m_S4B_APSPropellant;

    double m_LET_Mass;

    wxString m_LVDCSoftwareName;
};

class SettingsDialog : public wxPropertySheetDialog
{
	wxDECLARE_CLASS(SettingsDialog);
public:
	SettingsDialog(wxWindow* win, SettingsData& settingsData);
	~SettingsDialog();

    wxPanel* CreateGeneralSettingsPage(wxWindow* parent);
    wxPanel* CreateLaunchpadSettingsPage(wxWindow* parent);
    wxPanel* CreateMassesSettingsPage(wxWindow* parent);
    wxPanel* CreatePropulsionSettingsPage(wxWindow* parent);
protected:

    void LoadData();
    void SaveData();

    void EnableUpdate(bool enable = true);

    void OnApply(wxCommandEvent& event);
    void OnOK(wxCommandEvent& event);
    void OnCancel(wxCommandEvent& event);
    void OnTextChange(wxCommandEvent& event);
    void OnChoiceChange(wxCommandEvent& event);

    wxTextCtrl* CreateTextCtrl(wxPanel* panel, wxFlexGridSizer* flexGridSizer, const wxString& label, const wxString& unit, const wxString& tooltip) const;
    wxChoice* CreateChoice(wxPanel* panel, wxFlexGridSizer* flexGridSizer, const wxString& label, const wxString& tooltip, const wxArrayString& strings) const;

    wxTextCtrl* textLaunchTimeGMT;
    wxTextCtrl* textCountdownTime;
    wxChoice* choiceSaturnType;
    wxTextCtrl* textLVDCSoftwareName;

    wxTextCtrl* textLaunchpadLatitude;
    wxTextCtrl* textLaunchpadLongitude;
    wxTextCtrl* textLaunchpadAltitude;
    wxTextCtrl* textLaunchpadAzimuth;

    wxTextCtrl* textSI_EmptyMass;
    wxTextCtrl* textSI_MainFuelMass;
    wxTextCtrl* textSI_RetroFuelMass;
    wxTextCtrl* textSI_RetroNum;

    wxTextCtrl* textSII_EmptyMass;
    wxTextCtrl* textSII_MainFuelMass;
    wxTextCtrl* textSII_InterstageMass;
    wxTextCtrl* textSII_UllageFuelMass;
    wxTextCtrl* textSII_UllageNum;
    wxTextCtrl* textSII_RetroFuelMass;
    wxTextCtrl* textSII_RetroNum;

    wxTextCtrl* textS4B_EmptyMass;
    wxTextCtrl* textS4PL_Mass;
    wxTextCtrl* textS4B_MainFuelMass;
    wxTextCtrl* textS4B_UllageFuelMass;
    wxTextCtrl* textS4B_UllageCaseEmptyMass;
    wxTextCtrl* textS4B_UllageNum;
    wxTextCtrl* textS4B_APSPropellant;
    wxTextCtrl* textLET_Mass;

    SettingsData& m_settingsData;

    bool mDataChanged;

    enum {
        ID_TEXT_CTRL,
        ID_CHOICE
    };

	wxDECLARE_EVENT_TABLE();
};