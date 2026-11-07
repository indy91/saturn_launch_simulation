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

#include "LVDA.h"
#include "IU.h"
#include "LVDC.h"
#include "../../util/Constants.h"
#include "yaLVDC/yaLVDC.h"
#include "../Saturn.h"

#define TelemetryPIOMask 02 // 0202
#define LVDC_to_RCA_Interrupt_Mask 040000

inline int32_t bit_set_to(int32_t number, unsigned int n, bool x)
{
	return (number & ~((int32_t)1 << n)) | ((unsigned int)x << n);
}

LVDA::LVDA(IU* i) : iu(i)
{
	Attitude = AttitudeError = _V(0, 0, 0);
	LVDCPoweredOn = false;
	LastCycled = 0.0;

	TagRegister = 0;
	TelemetryBufferRegister = 0;
	DiscreteOutputRegister = 0;
	InternalControlRegister = 0;
}

int LVDA::SetMissionInfo(const std::string& LVDCSoftwareName)
{
	std::string FilePath = "..\\files\\" + LVDCSoftwareName;

	char* temp = (char*)FilePath.c_str();
	assemblies[numAssemblies++].name = temp;

	if (readAssemblies())
	{
		iu->GetSat()->SendEventMessage("Could not read LVDC files!");
		return 1;
	}

	state.lvdc_clientdata = this;
	state.hopSaver = 0;
	state.hop = 0;
	return 0;
}

void LVDA::Timestep(double simt, double simdt)
{
	// Process inputs

	// Discrete Inputs and Interrupts
	GetDiscreteInputs(state.pio[057]);
	GetInterrupts(state.pio[0137]);

	state.pio[077] = iu->GetControlDistributor()->GetSwitchSelectorFeedback();

	// Attitude
	Attitude = iu->GetLVIMU()->GetAttitude();

	state.pio[0223] = GimbalAngleToFineResolverWord(Attitude.x);
	state.pio[0363] = GimbalAngleToFineResolverWord(Attitude.y);
	state.pio[0353] = GimbalAngleToFineResolverWord(Attitude.z);
	state.pio[0243] = GimbalAngleToBackupResolverWord(Attitude.x);
	state.pio[0343] = GimbalAngleToBackupResolverWord(Attitude.y);
	state.pio[0233] = GimbalAngleToBackupResolverWord(Attitude.z);

	// Accelerometer
	VECTOR3 DV = iu->GetLVIMU()->GetAccumulatedVelocity();

	state.pio[0107] = ConvertAccelerometerData(DV.x);
	state.pio[0127] = ConvertAccelerometerData(DV.y);
	state.pio[0117] = ConvertAccelerometerData(DV.z);

	//iu->GetLVDC()->Timestep(simt);
	LVDCTimestep(simt, simdt);

	// Process outputs

	AttitudeError.x = ConvertAttitudeError(state.pio[0146]);
	AttitudeError.y = ConvertAttitudeError(state.pio[0152]);
	AttitudeError.z = ConvertAttitudeError(state.pio[0156]);
}

void LVDA::LVDCTimestep(double simt, double simdt)
{
	if (LVDCPoweredOn == false)
	{
		// Reset last cycling time
		LastCycled = 0;
		// Jump to 0
		state.hop = 0;

		return;
	}

	int Delta, CycleCount = 0;

	if (LastCycled == 0) {					// Use simdt as difference if new run
		LastCycled = (simt - simdt);
	}

	long cycles = (long)((simt - LastCycled) / SECONDS_PER_CYCLE); // Preserve the remainder

	while (CycleCount < cycles)
	{
		checkForInterrupts();

		if (runOneInstruction(&Delta))
		{
			double testval = 2.0;
			break;
		}

		if (processInterruptsAndIO())
		{
			double testval = 2.0;
		}

		/*
		if (state.pioChange != -1)
		{
			PIOOutput(state.pioChange, state.pio[state.pioChange]);
		}
		state.pioChange = -1;
		*/

		CycleCount += Delta;
	};

	LastCycled += (SECONDS_PER_CYCLE * CycleCount);
}

int32_t LVDA::ConvertAccelerometerData(double DV) const
{
	// INPUTS:
	// DV: Accumulated velocity in m/s
	// OUTPUTS:
	// Accelerometer PIO word

	double pulses;
	int32_t pulses2, pulses3;

	pulses = DV * 20.0; // 1 pulse = 0.05 m/s
	pulses2 = (int32_t)pulses;
	pulses3 = pulses2 % 07777;	// Modulo
	pulses3 = pulses3 & 07777;	// Limit

	return ((pulses3 << 14) + pulses3);
}

int32_t LVDA::GimbalAngleToBackupResolverWord(double Angle) const
{
	double ang2, ang3;

	ang2 = fmod(Angle, MathConstants::PI);
	ang3 = floor(ang2 * pow(2, 1) * 2016.0 / 360.0);
	return int32_t(ang3);
}

int32_t LVDA::GimbalAngleToFineResolverWord(double Angle) const
{
	double ang2, ang3;

	ang2 = fmod(Angle, 5.625 * MathConstants::RAD_PER_DEG);
	ang3 = floor(ang2 * pow(2, 6) * 2016.0 / 360.0);
	return int32_t(ang3);
}

double LVDA::ConvertAttitudeError(int32_t err) const
{
	// 1 bit = 0.06 degrees
	// Input is 9 bits with sign
	// Output in radians

	if ((0400 & err) != 0)
	{
		err |= ~0777;
	}

	return 0.06 * MathConstants::RAD_PER_DEG * (double)(err);
}

int LVDA::ConvertPIOToTag(int pio) const
{
	int tag = 0200; // Bit 8 for LVDC

	// Option 1: T8, 1, 2, 7, 6, 5, 4, 3
	if (pio & 0200) tag |= 0100; // A8
	if (pio & 0400) tag |= 040; // A9

	tag += ((pio & 0174) >> 2); // A7-A3

	// Option2 :
	/*
	tag += (pio & 0174); // Bits 3-7

	if (pio & 0400) tag += 2; // Bit 2
	if (pio & 0200) tag += 1; // Bit 1
	*/

	return tag;
}

void LVDA::PIOProcessing(int pio, int32_t data)
{
	// Is it telemetry?
	if (((~pio) & TelemetryPIOMask) == TelemetryPIOMask)
	{
		TagRegister = ConvertPIOToTag(pio);
		TelemetryBufferRegister = data; // (data & 0377777777);

		// Send to GUI
		// char Buffer[64];
		// sprintf(Buffer, "PIO %03o %o", pio, data);
		// iu->GetSat()->SendEventMessage(Buffer);
	}

	switch (pio)
	{
	case 012: // Discrete Output Register (Reset)
		DiscreteOutputRegister = DiscreteOutputRegister & (~data);
		if ((data & LVDC_to_RCA_Interrupt_Mask) != 0)
		{
			// Reset interrupt to RCA-110A
			iu->GetIUToIUESECommandConnector()->LVDC_To_RCA110_Interrupt(false);
		}
		break;
	case 016: // Discrete Output Register (Set)
		DiscreteOutputRegister = (DiscreteOutputRegister | data);
		if ((DiscreteOutputRegister & LVDC_to_RCA_Interrupt_Mask) != 0)
		{
			// Set interrupt to RCA-110A
			iu->GetIUToIUESECommandConnector()->LVDC_To_RCA110_Interrupt(true);
		}
		break;
	case 022: // Internal Control Register (Set)
		InternalControlRegister = (InternalControlRegister | data);
		break;
	case 026: // Internal Control Register (Reset)
		InternalControlRegister = InternalControlRegister & (~data);
		break;
	case 036: // Switch Selector Register (Load)
		iu->GetControlDistributor()->SwitchSelector(data);
		// Also update fedback register
		state.pio[077] = iu->GetControlDistributor()->GetSwitchSelectorFeedback();
		break;
	}
}

bool LVDA::GetGuidanceReferenceRelease()
{
	return iu->GetIUToIUESECommandConnector()->GetGuidanceReferenceRelease();
}

bool LVDA::GetLiftoff()
{
	return iu->GetEDS()->GetIULiftoffSignal();
}

bool LVDA::GetSIPropellantDepletionEngineCutoff()
{
	return iu->GetSIPropellantDepletionEngineCutoff();
}

bool LVDA::GetSIIPropellantDepletionEngineCutoff()
{
	return iu->GetSIIPropellantDepletionEngineCutoff();
}

bool LVDA::GetSIVBEngineOutA()
{
	return iu->GetControlDistributor()->GetSIVBEngineOutA();
}

bool LVDA::GetSIVBEngineOutB()
{
	return iu->GetControlDistributor()->GetSIVBEngineOutB();
}

VECTOR3 LVDA::GetAttitude()
{
	return Attitude;
}

int LVDA::GetPIO(int num) const
{
	return state.pio[num];
}

void LVDA::WritePIO(int num, int32_t data)
{
	// Only for stand-in LVDC
	state.pio[num] = data;
	PIOProcessing(num, data);
}

void LVDA::SendData(int data)
{
	state.pio[043] = ((data << 12) & 0377777777); // input is 14 bits. Left shift 12 bits.
}

void LVDA::SwitchLVDCPowerOn()
{
	LVDCPoweredOn = true;
}

void LVDA::SwitchLVDCPowerOff()
{
	LVDCPoweredOn = false;
}

bool LVDA::GetSyncReset()
{
	// Discrete output 1/LVDA word bit 12
	return ((DiscreteOutputRegister & 020000) != 0);
}

/*
bool LVDA::GetRCA110Interrupt()
{
	// Discrete output 2/LVDA word bit 11
	return ((DiscreteOutputRegister & LVDC_to_RCA_Interrupt_Mask) != 0);
}
*/

bool LVDA::GetGuidanceReferenceFailureA()
{
	// Discrete output 4/LVDA word bit 9
	return ((DiscreteOutputRegister & 0200000) != 0);
}

bool LVDA::GetGuidanceReferenceFailureB()
{
	// Discrete output 6/LVDA word bit 7
	return ((DiscreteOutputRegister & 01000000) != 0);
}

bool  LVDA::GetFiringCommitEnable()
{
	// Discrete output 12/LVDA word bit 1
	return ((DiscreteOutputRegister & 0100000000) != 0);
}

bool  LVDA::GetFiringCommitInhibit()
{
	// Discrete output 13/LVDA word sign
	return ((DiscreteOutputRegister & 0200000000) != 0);
}

int LVDA::GetModeRegister()
{
	return ((state.pio[06] & 0374000000) >> 20);
}

void LVDA::GetHardwires(int& tag, int& telemetrybuffer)
{
	tag = TagRegister;
	telemetrybuffer = TelemetryBufferRegister;
}

VECTOR3 LVDA::GetAttitudeError()
{
	return AttitudeError;
}

LVDA1B::LVDA1B(IU* i) : LVDA(i)
{

}

void LVDA1B::GetInterrupts(int32_t& pio)
{

}

void LVDA1B::GetDiscreteInputs(int32_t& pio)
{

}

LVDASV::LVDASV(IU* i) : LVDA(i)
{

}

void LVDASV::GetInterrupts(int32_t& pio)
{
	// Bits 0-13 not used
	// Bit 14: Spare

	// TBD: Careful this is mission specific
	pio = bit_set_to(pio, 15, iu->GetEDS()->GetSIISIVBSepSeqStart());
	// Prior missions:
	//pio = bit_set_to(pio, 15, iu->GetSIInboardEngineOut());
	pio = bit_set_to(pio, 16, iu->GetIUToIUESECommandConnector()->GetRCA110Interrupt());
	pio = bit_set_to(pio, 17, iu->GetControlDistributor()->GetSIVBEngineOutB());
	pio = bit_set_to(pio, 18, iu->GetSIPropellantDepletionEngineCutoff()); // S-IC Outboard Engines Cutoff A
	pio = bit_set_to(pio, 19, iu->GetSIIEnginesOut());
	pio = bit_set_to(pio, 20, iu->GetIUToIUESECommandConnector()->GetGuidanceReferenceRelease());
}

void LVDASV::GetDiscreteInputs(int32_t& pio)
{
	// Bits 0 and 1 not used
	pio = bit_set_to(pio, 2, iu->GetIUToIUESECommandConnector()->GetRCA110Sync());
	// TBD: Bit 3: Command Decoder
	pio = bit_set_to(pio, 4, iu->GetSIVBCommandConnector()->GetSIVBBurnerMalfunction());
	pio = bit_set_to(pio, 5, iu->GetEDS()->GetSIVBTakeover());
	pio = bit_set_to(pio, 6, iu->GetControlDistributor()->GetSIVBEngineOutA());

	pio = bit_set_to(pio, 7, iu->GetEDS()->GetTLIInhibit());
	// Spare on earlier missions, redundant liftoff on Apollo 17 and later
	pio = bit_set_to(pio, 8, iu->GetEDS()->GetIULiftoffSignal());
	// Bit 9: Spare
	pio = bit_set_to(pio, 10, iu->GetEDS()->GetSIVBTakeover());
	pio = bit_set_to(pio, 11, iu->GetSIVBCommandConnector()->SIXSIVBNotSeparated());
	pio = bit_set_to(pio, 12, iu->GetSIInboardEngineOut());
	pio = bit_set_to(pio, 13, iu->GetSIVBCommandConnector()->SICSIINotSeparated());
	pio = bit_set_to(pio, 14, iu->GetSIIInboardEngineOut());
	pio = bit_set_to(pio, 15, iu->GetSIOutboardEngineOut());
	pio = bit_set_to(pio, 16, iu->GetSIVBCommandConnector()->SIIInterstageNotSeparated());
	pio = bit_set_to(pio, 17, iu->GetIUToIUESECommandConnector()->GetGuidanceReferenceReleaseAlert());
	pio = bit_set_to(pio, 18, false); // TBD: Later missions: Upper Outboard engine out, earlier missions: iu->GetEDS()->GetSIISIVBSepSeqStart());
	pio = bit_set_to(pio, 19, iu->GetSIPropellantDepletionEngineCutoff());
	pio = bit_set_to(pio, 20, iu->GetSIIEnginesOut());
	pio = bit_set_to(pio, 21, iu->GetEDS()->GetSIVBIgnitionSequenceStart());
	pio = bit_set_to(pio, 22, iu->GetSIIOutboardEngineOut());

	pio = bit_set_to(pio, 23, iu->GetEDS()->GetSIISIVBSepSeqStart()); // iu->GetSIPropellantDepletionEngineCutoff());
	// Later missions:
	pio = bit_set_to(pio, 24, iu->GetEDS()->GetCMCSIVBCutoff());
	pio = bit_set_to(pio, 25, iu->GetEDS()->GetIULiftoffSignal());
}

int pendingVirtualWireActivity(void)
{
	if (state.pioChange != -1)
	{
		LVDA* lvda;
		int channel, payload;

		lvda = (LVDA*)state.lvdc_clientdata;

		channel = state.pioChangeFull;
		payload = state.pio[state.pioChange];

		lvda->PIOProcessing(channel, payload);
	}

	return 0;
}