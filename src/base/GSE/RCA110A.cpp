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

#include "RCA110A.h"
#include "SI_ESE.h"
#include "IU_ESE.h"
#include "../../util/Constants.h"

RCA110A::RCA110A(double a_zp) : A_ZP(a_zp)
{
	si_ese = NULL;
	iu_ese = NULL;
	bLVDC_Interrupt = false;
	bSync = false;
	bInterruptLast = false;
	bSyncReset = false;
	bInterrupt = false;
	program = 0;
	State = 0;
	SubState = 0;
	StateTime = 0.0;
	SubStateTime = 0.0;
	LastTag = 0;
	LastPIOValue = 0;
	StatusCode = 0;
	GMT = 0.0;
	GMTLO = 0.0;
	for (int i = 0; i < 20; i++)
	{
		iStorage[i] = 0;
		dStorage[i] = 0.0;
	}
	for (int i = 0; i < 12; i++)
	{
		LVDCHistoryTable[i] = 0;
	}
	LVDCErrorMessage = 0;
	CLTIME = 0.0;
}

void RCA110A::Init(SI_ESE* siese, IU_ESE* iuese, double gmtlo)
{
	si_ese = siese;
	iu_ese = iuese;
	GMTLO = gmtlo;
}

void RCA110A::Timestep(double gmt, double simdt)
{
	// Store
	GMT = gmt;

	// Process input signals
	bSyncReset = iu_ese->GetIUESEToIUCommandConnector()->GetRCA110SyncReset();

	ProcessTestPrograms(simdt);

	// Reset SYNC
	if (bSyncReset && bSync)
	{
		bSync = false;
	}

	// Peripheral test
	//iu_ese->GetIUESEToIUCommandConnector()->SendLVDCData(0);
}

bool RCA110A::GetInterrupt()
{
	return bLVDC_Interrupt;
}

bool RCA110A::GetSync()
{
	return bSync;
}

void RCA110A::SetInterrupt(int set)
{
	bInterrupt = set;

	if (bInterrupt && !bInterruptLast)
	{
		// Save time
		CLTIME = GMT;

		// Get mode register
		int iTemp = iu_ese->GetIUESEToIUCommandConnector()->GetLVDAModeRegister();

		if (iTemp != LVDCHistoryTable[0])
		{
			// Update LVDC history table
			for (int i = 10; i >= 0; i--)
			{
				LVDCHistoryTable[i + 1] = LVDCHistoryTable[i];
			}
			LVDCHistoryTable[0] = iTemp;
		}

		// Read telemetry register
		iu_ese->GetIUESEToIUCommandConnector()->GetLVDAHardwires(LastTag, LastPIOValue);

		// Special PIO processing
		switch (LastTag)
		{
		case 0356: // Status Code
			StatusCode = (LastPIOValue >> 20); // Really?
			break;
		case 0171: // Error
			//LVDCErrorMessage = LastPIOValue;
			break;
		}

		// Issue sync
		bSync = true;
	}

	bInterruptLast = bInterrupt;
}

void RCA110A::StartFT04LVDCPowerUp()
{
	program = FT04;
	programName = "FT04";
	setState(0);
}

void RCA110A::StartFT05DataLoadTest()
{
	program = FT05;
	programName = "FT05";
	setState(0);
}

void RCA110A::StartFT42PrepareToLaunch()
{
	program = FT42;
	programName = "FT42";
	setState(0);
}

void RCA110A::StartGT16AzimuthLaying(double azi)
{
	// Azimuth in degrees

	program = GT16;
	programName = "GT16";
	setState(0);

	dStorage[0] = azi;
}

void RCA110A::ProcessTestPrograms(double simdt)
{
	StateTime += simdt;
	SubStateTime += simdt;

	switch (program)
	{
	case FT04:
		FT04LVDCPowerUp();
		break;
	case FT05:
		FT05DataLoadTest();
		break;
	case FT42:
		FT42PrepareToLaunch();
		break;
	case GT16:
		GT16AzimuthLaying();
		break;
	}
}

void RCA110A::FT04LVDCPowerUp()
{
	switch (State)
	{
	case 0:
		iu_ese->GetIUESEToIUCommandConnector()->SetLVDCPower(true);
		setState(State + 1);
		break;
	case 1: // Wait for mode 26
		if (bSync == false && LVDCHistoryTable[0] == 026 && bSyncReset == false)
		{
			bSync = true;
			setState(State + 1);
		}
		break;
	case 2: // Wait for mode sort (mode 0)
		if (LVDCHistoryTable[0] == 0)
		{
			setState(State + 1);
		}
		break;
	default:
		endProgram();
		break;
	}
}

void RCA110A::FT05DataLoadTest()
{
	switch (State)
	{
	case 0:
		// Test values
		iStorage[10] = 99;
		iStorage[11] = 42;
		setState(State + 1);
		break;
	case 1: // Start mode 21
		SCMND(021, State + 1, 100);
		break;
	case 2:
		SWRTE(0, 016, 0371, 2, iStorage + 10, State + 1, 100);
		break;
	case 100: // Error return
		break;
	default:
		endProgram();
		break;
	}
}

void RCA110A::FT42PrepareToLaunch()
{
	switch (State)
	{
	case 0:
		// Save entry Time to PTL
		dStorage[0] = GMT;
		setState(State + 1);
	case 1:
		// Command LVDC to routine 12 (PTL)
		SCMND(012, State + 1, 100);
		break;
	case 2: // Wait for LVDC to be in routine 12
		if (LVDCHistoryTable[0] == 012)
		{
			setState(State + 1);
		}
		break;
	case 3: // Normal return from SCMND
		// Load GMT
		iStorage[0] = (int)(CLTIME * pow(2, 9));
		setState(State + 1);
	case 4:
		SWRTE(0, 016, 0371, 1, iStorage, State + 1, 100);
		break;
	case 5: // Load PLOT
		iStorage[0] = (int)(GMTLO * pow(2, 9));
		setState(State + 1);
	case 6:
		SWRTE(0, 016, 0374, 1, iStorage, State + 1, 100);
		break;
	case 7: // 
		break;
	case 100: // Error return
		break;
	default:
		endProgram();
		break;
	}
}

void RCA110A::GT16AzimuthLaying()
{
	switch (State)
	{
	case 0:
		PlatformAzimuthAlignment(dStorage[0]);
		setState(State + 1);
		break;
	case 1:
		endProgram();
		break;
	}
}

int RCA110A::GetStatusCode()
{
	return StatusCode;
}

void RCA110A::GetLVDAData(int& tag, int& data)
{
	tag = LastTag;
	data = LastPIOValue;
}

std::string RCA110A::GetProgram()
{
	return programName;
}

int RCA110A::GetState()
{
	return State;
}

int RCA110A::GetSubState()
{
	return SubState;
}

int RCA110A::GetLVDCErrorMessage()
{
	return LVDCErrorMessage;
}

void RCA110A::FormatLVDCLoad(int sector, int module, int address, int data, bool IsLast, bool VerifyOnly)
{
	// data is 26 bits and will be split up over 3 of the words
	int datawords[3];

	// Most significant bits first
	datawords[0] = ((data >> 22) & 017);
	datawords[1] = ((data >> 11) & 03777);
	datawords[2] = (data & 03777);

	// Word 1
	iStorage[0] = 0;

	iStorage[0] = iStorage[0] | ((sector & 017) << 9); // Sector in bits 1-4
	iStorage[0] = iStorage[0] | ((module & 07) << 6); // Module in bits 5-7
	iStorage[0] = iStorage[0] | ((datawords[0] & 017) << 2); // Data in bits 8-11

	// Word 2
	iStorage[1] = 1;
	iStorage[1] = iStorage[1] | ((address & 0377) << 5); // Address in bits 1-8
	if (IsLast)
	{
		iStorage[1] = iStorage[1] | 020;
	}
	if (VerifyOnly)
	{
		iStorage[1] = iStorage[1] | 010;
	}

	// Word 3
	iStorage[2] = 2;
	iStorage[2] = iStorage[2] | (datawords[1] << 2);

	// Word 4
	iStorage[3] = 3;
	iStorage[3] = iStorage[3] | (datawords[2] << 2);
}

void RCA110A::SendLVDCData(int data)
{
	iu_ese->GetIUESEToIUCommandConnector()->SendLVDCData(data & 037777); // Up to 14 bits
	bSync = true;
}

void RCA110A::PlatformAzimuthAlignment(double A_Z)
{
	double roll_angle;

	roll_angle = A_Z - A_ZP;

	if (roll_angle < 0.0) roll_angle += 360.0;

	roll_angle = roll_angle * MathConstants::RAD_PER_DEG;

	iu_ese->GetIUESEToIUCommandConnector()->AzimuthAlignment(roll_angle);
}

void RCA110A::SCMND(int XX, int NormalReturn, int ErrorReturn)
{
	switch (SubState)
	{
	case 0:
		SendLVDCData((XX << 6) | 020000); // Set sign bit for mode
		setSubState(SubState + 1);
		break;
	case 1:
		if (bSync == false)
		{
			setSubState(SubState + 1);
		}
		if (SubStateTime > 100.0)
		{
			// Error
			setState(ErrorReturn);
		}
		break;
	case 2:
		if (LVDCHistoryTable[0] != XX)
		{
			setState(ErrorReturn);
		}
		else
		{
			setSubState(SubState + 1);
		}
		break;
	case 3:
		// End of subroutine
		setState(NormalReturn);
		break;
	default:
		setState(ErrorReturn);
		break;
	}
}

void RCA110A::SWRTE(int sector, int module, int address, int NUM, int* VALS, int NormalReturn, int ErrorReturn)
{
	switch (SubState)
	{
	case 0:
		// Counter
		iStorage[9] = 0;
		setSubState(SubState + 1);
		break;
	case 1:
		// Format data word
		FormatLVDCLoad(sector, module, address + iStorage[9], VALS[iStorage[9]], (iStorage[9] + 1) >= NUM, false);
		// Send first one
		SendLVDCData(iStorage[0]);
		setSubState(SubState + 1);
		break;
	case 2:
	case 3:
	case 4:
		if (bSync == false)
		{
			SendLVDCData(iStorage[SubState - 1]);
			setSubState(SubState + 1);
		}
		if (SubStateTime > 100.0)
		{
			// Error
			setState(ErrorReturn);
		}
		break;
	case 5: // Done
		if (bSync == false)
		{
			iStorage[9]++;
			if (iStorage[9] >= NUM)
			{
				setSubState(SubState + 1);
			}
			else
			{
				// Back to substep 1
				setSubState(1);
			}
		}
		break;
	case 6: // Verification
		break;
	default:
		setState(ErrorReturn);
		break;
	}
}

void RCA110A::endProgram()
{
	program = 0;
	programName = "";
	setState(0);
}

void RCA110A::setState(int newState)
{
	State = newState;
	SubState = 0;
	StateTime = 0.0;
	SubStateTime = 0.0;
}

void RCA110A::setSubState(int newState)
{
	SubState = newState;
	SubStateTime = 0.0;
}