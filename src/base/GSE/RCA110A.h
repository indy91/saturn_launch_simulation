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

#include <string>

class SI_ESE;
class IU_ESE;

class RCA110A
{
public:
	RCA110A(double a_zp);

	void Init(SI_ESE* siese, IU_ESE* iuese, double gmtlo);

	void Timestep(double gmt, double simdt);

	// From ML
	double GetGMTLO() { return GMTLO; }

	// From LVDA
	bool GetInterrupt();
	bool GetSync();
	void SetInterrupt(int set);

	// Test programs
	void StartFT04LVDCPowerUp();
	void StartFT05DataLoadTest();
	void StartFT42PrepareToLaunch();
	void StartGT16AzimuthLaying(double azi);

	// To GUI
	int GetModeRegister() { return LVDCHistoryTable[0]; }
	int GetStatusCode();
	void GetLVDAData(int& tag, int& data);
	std::string GetProgram();
	int GetState();
	int GetSubState();
	int GetLVDCErrorMessage();
protected:
	void ProcessTestPrograms(double simdt);

	// Test Programs
	void FT04LVDCPowerUp();
	void FT05DataLoadTest();
	void FT42PrepareToLaunch();
	void GT16AzimuthLaying();

	void FormatLVDCLoad(int sector, int module, int address, int data, bool IsLast, bool VerifyOnly);
	void SendLVDCData(int data);

	void PlatformAzimuthAlignment(double A_Z);

	// LVDC macros
	// Command LVDC to Status XX
	void SCMND(int XX, int NormalReturn, int ErrorReturn);
	// Send Data to LVDC
	void SWRTE(int sector, int module, int address, int NUM, int* VALS, int NormalReturn, int ErrorReturn);
	// Read 1 LVDC word
	//void READ();
	// Set SYNC X Times
	//void SYNC(int X);
	// Set Sync One Time, Do Not Wait for Reset
	//void SYNCB();
	// Reset Sync
	//void RSYNC();

	void endProgram();
	void setState(int newState);
	void setSubState(int newState);

	// Input signals
	int LVDCHistoryTable[12];
	int LastTag;
	int LastPIOValue;
	bool bSyncReset;
	bool bInterrupt;

	// Outputs signals
	bool bLVDC_Interrupt;
	bool bSync;

	bool bInterruptLast;

	// Program
	std::string programName;
	int program; // 0 = None, 1 = PTL
	int State;
	double StateTime;
	int SubState;
	double SubStateTime;

	// Data
	double GMT;
	double GMTLO;
	int iStorage[20];
	double dStorage[20];
	int LVDCErrorMessage;
	int StatusCode;
	double CLTIME;			// LVDC Time Constant, interpreted as time of last LVDC interrupt

	SI_ESE* si_ese;
	IU_ESE* iu_ese;

	const double A_ZP;	// Launchpad azimuth, degrees

	enum Programs
	{
		FT04 = 1,
		FT05,
		FT42,
		GT16
	};
};