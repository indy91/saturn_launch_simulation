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

#include "IUUmbilical.h"
#include "../IU/LVDA.h"
#include "IU_ESE.h"
#include "../../util/Constants.h"

IU_ESE::IU_ESE()
{
	iuESEToIUCommandConnector.SetIU_ESE(this);

	CommandVehicleLiftoffIndicationInhibit = true;
	ThrustOKIndicateEnableInhibitA = true;
	ThrustOKIndicateEnableInhibitB = true;
	EDSLiftoffInhibitA = true;
	EDSLiftoffInhibitB = true;
	PadAbortRequest = false;
	EDSPowerInhibit = false;
	SIBurnModeSubstitute = false;
	GuidanceReferenceRelease = false;
	GuidanceReferenceReleaseAlert = false;
	EDSNotReady = false;
	InstrumentUnitReady = false;

	FCCPowerIsOn = false;
	QBallSimulateCmd = false;
	SwitchFCCPowerOn = false;
	SwitchFCCPowerOff = false;
	OneEngineOutA = false;
	SCCutoffEnableA = false;
	SCCutoffEnableB = false;
	LiftoffReset = false;
	LiftoffEnableA = false;
	LiftoffEnableB = false;

	for (int i = 0; i < 9; i++)
	{
		OverrateSimulate[i] = false;
	}
	for (int i = 0; i < 6; i++)
	{
		EDSAutoAbortSimulate[i] = false;
		EDSAbortCommandToSC[i] = false;
	}
	for (int i = 0; i < 3; i++)
	{
		EDSCutoffFromSC[i] = false;
		ExcessivePitchYawRateAutoAbortInhibit[i] = false;
		ExcessiveRollRateAutoAbortInhibit[i] = false;
		TwoEngineOutAutoAbortInhibit[i] = true; //Initialize this to true to prevent race condition with EDS. And it's normally inhibited anyway.
	}

	LastMissionTime = 10000000.0;

	rca110 = NULL;
}

IU_ESE::~IU_ESE()
{
	iuESEToIUCommandConnector.Disconnect();
}

void IU_ESE::Init(RCA110A* rca)
{
	rca110 = rca;
}

void IU_ESE::Timestep(double MissionTime, double simdt)
{
	//FCC Power (MDI 0861, MDO 1823)
	FCCPowerIsOn = iuESEToIUCommandConnector.FCCPowerIsOn();

	SwitchFCCPowerOn = true; // TBD

	//TBD: Get analog signal from LCC
	SwitchFCCPowerOff = false;

	if (SwitchFCCPowerOn && !SwitchFCCPowerOff)
		iuESEToIUCommandConnector.SwitchFCCPowerOn();
	if (SwitchFCCPowerOff || (!SwitchFCCPowerOn && !SwitchFCCPowerOff))
		iuESEToIUCommandConnector.SwitchFCCPowerOff();

	//Telemetry
	OneEngineOutA = !iuESEToIUCommandConnector.AllSIEnginesRunning();
	SCCutoffEnableA = iuESEToIUCommandConnector.GetSCCutoffEnabledA();
	SCCutoffEnableB = iuESEToIUCommandConnector.GetSCCutoffEnabledB();
	LiftoffEnableA = iuESEToIUCommandConnector.GetLiftoffEnableA();
	LiftoffEnableB = iuESEToIUCommandConnector.GetLiftoffEnableB();
	LiftoffReset = !LiftoffEnableA && !LiftoffEnableB;
	iuESEToIUCommandConnector.GetEDSAbortCommandToSC(EDSAbortCommandToSC);

	//EDS Test
	if ((MissionTime >= -6900.0) && (LastMissionTime < -6900.0))
	{
		//Abort Light On
		SetEDSMode(LCC_EDS_MODE_TEST);
		PadAbortRequest = true;
	}
	else if ((MissionTime >= -6840.0) && (LastMissionTime < -6840.0))
	{
		//Abort Light Off
		SetEDSMode(LCC_EDS_MODE_TEST);
		PadAbortRequest = false;
	}
	else if ((MissionTime >= -6780.0) && (LastMissionTime < -6780.0))
	{
		//LV engine indicators on
		SetEDSMode(LCC_EDS_MODE_TEST);
		ThrustOKIndicateEnableInhibitA = false;
		ThrustOKIndicateEnableInhibitB = false;
	}
	else if ((MissionTime >= -6720.0) && (LastMissionTime < -6720.0))
	{
		//LV rate light on
		OverrateSimulate[0] = true;
		OverrateSimulate[1] = true;
		OverrateSimulate[2] = true;
	}
	else if ((MissionTime >= -6660.0) && (LastMissionTime < -6660.0))
	{
		OverrateSimulate[0] = false;
		OverrateSimulate[1] = false;
		OverrateSimulate[2] = false;
		SetEDSMode(LCC_EDS_MODE_OFF);
	}
	else if ((MissionTime >= -6660.0) && (MissionTime < -390.0))
	{
		SetEDSMode(LCC_EDS_MODE_OFF);

	}
	else if ((MissionTime >= -390.0) && (MissionTime < -250.0))
	{
		SetEDSMode(LCC_EDS_MODE_LAUNCH);
	}
	else if (MissionTime >= -250.0)
	{
		SetEDSMode(LCC_EDS_MODE_LAUNCH);
		ThrustOKIndicateEnableInhibitA = false;
		ThrustOKIndicateEnableInhibitB = false;
		SIBurnModeSubstitute = true;

		EDSNotReady = iuESEToIUCommandConnector.IsEDSUnsafeA() || iuESEToIUCommandConnector.IsEDSUnsafeB() || !iuESEToIUCommandConnector.GetEDSSCCutoff1() || !iuESEToIUCommandConnector.GetEDSSCCutoff2() || !iuESEToIUCommandConnector.GetEDSSCCutoff3();
		EDSNotReady = EDSNotReady || iuESEToIUCommandConnector.GetEDSAutoAbortBus() || iuESEToIUCommandConnector.GetEDSExcessiveRollRateIndication() || iuESEToIUCommandConnector.GetEDSExcessivePitchYawRateIndication();

		InstrumentUnitReady = iuESEToIUCommandConnector.GetFiringCommitEnable() && !iuESEToIUCommandConnector.GetFiringCommitInhibit() && !iuESEToIUCommandConnector.GetGuidanceReferenceFailureA()
			&& !iuESEToIUCommandConnector.GetGuidanceReferenceFailureB() && !EDSNotReady;
	}

	LastMissionTime = MissionTime;
}

void IU_ESE::SetEDSMode(int mode)
{
	//Off
	if (mode == LCC_EDS_MODE_OFF)
	{
		EDSPowerInhibit = true;
		EDSLiftoffInhibitA = true;
		EDSLiftoffInhibitB = true;
		for (int i = 0; i < 9; i++)
		{
			OverrateSimulate[i] = false;
		}
		ThrustOKIndicateEnableInhibitA = true;
		ThrustOKIndicateEnableInhibitB = true;
		PadAbortRequest = false;
		iuESEToIUCommandConnector.EDSLiftoffEnableReset();
	}
	//Monitor
	else if (mode == LCC_EDS_MODE_MONITOR)
	{
		ExcessiveRollRateAutoAbortInhibit[0] = true;
		ExcessiveRollRateAutoAbortInhibit[1] = true;
		ExcessiveRollRateAutoAbortInhibit[2] = true;
		ExcessivePitchYawRateAutoAbortInhibit[0] = true;
		ExcessivePitchYawRateAutoAbortInhibit[1] = true;
		ExcessivePitchYawRateAutoAbortInhibit[2] = true;
		EDSPowerInhibit = false;
		iuESEToIUCommandConnector.EDSLiftoffEnableReset();
	}
	//Test
	else if (mode == LCC_EDS_MODE_TEST)
	{
		ExcessiveRollRateAutoAbortInhibit[0] = true;
		ExcessiveRollRateAutoAbortInhibit[1] = true;
		ExcessiveRollRateAutoAbortInhibit[2] = true;
		ExcessivePitchYawRateAutoAbortInhibit[0] = true;
		ExcessivePitchYawRateAutoAbortInhibit[1] = true;
		ExcessivePitchYawRateAutoAbortInhibit[2] = true;
		EDSPowerInhibit = false;
		iuESEToIUCommandConnector.EDSLiftoffEnableReset();
	}
	//Launch
	else if (mode == LCC_EDS_MODE_LAUNCH)
	{
		ExcessiveRollRateAutoAbortInhibit[0] = true;
		ExcessiveRollRateAutoAbortInhibit[1] = true;
		ExcessiveRollRateAutoAbortInhibit[2] = true;
		ExcessivePitchYawRateAutoAbortInhibit[0] = true;
		ExcessivePitchYawRateAutoAbortInhibit[1] = true;
		ExcessivePitchYawRateAutoAbortInhibit[2] = true;
		EDSPowerInhibit = false;
	}
}

bool IU_ESE::GetEDSAutoAbortSimulate(int n)
{
	if (n >= 1 && n <= 6)
	{
		return EDSAutoAbortSimulate[n - 1];
	}
	return false;
}

bool IU_ESE::GetEDSLVCutoffSimulate(int n)
{
	if (n >= 1 && n <= 3)
	{
		return EDSCutoffFromSC[n - 1];
	}
	return false;
}

bool IU_ESE::GetOverrateSimulate(int n)
{
	if (n >= 1 && n <= 9)
	{
		return OverrateSimulate[n - 1];
	}
	return false;
}

bool IU_ESE::GetExcessiveRollRateAutoAbortInhibit(int n)
{
	if (n >= 1 && n <= 3)
	{
		return ExcessiveRollRateAutoAbortInhibit[n - 1];
	}
	return false;
}

bool IU_ESE::GetExcessivePitchYawRateAutoAbortInhibit(int n)
{
	if (n >= 1 && n <= 3)
	{
		return ExcessivePitchYawRateAutoAbortInhibit[n - 1];
	}
	return false;
}

bool IU_ESE::GetTwoEngineOutAutoAbortInhibit(int n)
{
	if (n >= 1 && n <= 3)
	{
		return TwoEngineOutAutoAbortInhibit[n - 1];
	}
	return false;
}

IUSV_ESE::IUSV_ESE()
{
	SICOutboardEnginesCantInhibit = false;
	SICOutboardEnginesCantSimulate = false;
}