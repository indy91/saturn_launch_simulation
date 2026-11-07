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

#include "../../util/VectorMath.h"
#include <cstdint>
#include <string>

#define SWITCH_SELECTOR_IU 0
#define SWITCH_SELECTOR_SI 1
#define SWITCH_SELECTOR_SII 2
#define SWITCH_SELECTOR_SIVB 3

enum LVDCOutputRegister {

	ResetCommandDecoder = 1,
	RCA110AInterrupt,
	GuidanceReferenceFailureA = 3,
	GuidanceReferenceFailureB = 5,
	FiringCommitEnable = 11,
	FiringCommitInhibit,
};

class IU;

class LVDA
{
public:
	LVDA(IU* i);

	int SetMissionInfo(const std::string& LVDCSoftwareName);

	void Timestep(double simt, double simdt);

	// LVDC to LVDA
	void PIOProcessing(int pio, int32_t data);

	bool GetGuidanceReferenceRelease();
	bool GetLiftoff();
	bool GetSIPropellantDepletionEngineCutoff();
	bool GetSIIPropellantDepletionEngineCutoff();
	bool GetSIVBEngineOutA();
	bool GetSIVBEngineOutB();
	VECTOR3 GetAttitude();

	// LVDA to LVDC
	int GetPIO(int num) const;
	void WritePIO(int num, int32_t data);

	// IU ESE to LVDA
	void SendData(int data);
	void SwitchLVDCPowerOn();
	void SwitchLVDCPowerOff();

	// LVDA to IU ESE
	bool GetSyncReset();
	bool GetGuidanceReferenceFailureA();
	bool GetGuidanceReferenceFailureB();
	bool GetFiringCommitEnable();
	bool GetFiringCommitInhibit();

	int GetModeRegister();
	void GetHardwires(int& tag, int& telemetrybuffer);

	VECTOR3 GetAttitudeError();

	virtual void GetInterrupts(int32_t& pio) = 0;
	virtual void GetDiscreteInputs(int32_t& pio) = 0;
protected:

	void LVDCTimestep(double simt, double simdt);

	int32_t ConvertAccelerometerData(double DV) const;
	int32_t GimbalAngleToBackupResolverWord(double Angle) const;
	int32_t GimbalAngleToFineResolverWord(double Angle) const;

	double ConvertAttitudeError(int32_t err) const;

	int ConvertPIOToTag(int pio) const;

	IU* iu;
	VECTOR3 Attitude;
	VECTOR3 AttitudeError;

	// Relays
	bool LVDCPoweredOn;

	// LVDC
	//state_t LVDC;

	double LastCycled;

	// LVDA registers
	int TagRegister;				// 8 bits, for now saves PIO
	int TelemetryBufferRegister;	// 26 bits
	int DiscreteOutputRegister;		// 13 bits
	int InternalControlRegister;	// 13 bits
};

class LVDA1B : public LVDA
{
public:
	LVDA1B(IU* i);

	void GetInterrupts(int32_t& pio);
	void GetDiscreteInputs(int32_t& pio);
};

class LVDASV : public LVDA
{
public:
	LVDASV(IU* i);

	void GetInterrupts(int32_t& pio);
	void GetDiscreteInputs(int32_t& pio);
};