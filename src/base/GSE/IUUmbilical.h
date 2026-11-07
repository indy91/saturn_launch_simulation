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

#include "../connector.h"

class IU_ESE;

//IU ESE to IU connector
class IUESEToIUCommandConnector : public Connector
{
public:
	IUESEToIUCommandConnector();
	~IUESEToIUCommandConnector();

	//IU to IU ESE
	bool ReceiveMessage(Connector* from, ConnectorMessage& m);

	//IU ESE to IU
	void SetEDSLiftoffEnableA();
	void SetEDSLiftoffEnableB();
	void EDSLiftoffEnableReset();
	void SwitchFCCPowerOn();
	void SwitchFCCPowerOff();
	void SwitchQBallPowerOn();
	void SwitchQBallPowerOff();
	void SetControlSignalProcessorPower(bool set);
	void EDSGroupNo1Reset();
	void EDSGroupNo2Reset();
	bool AllSIEnginesRunning();
	bool IsEDSUnsafeA();
	bool IsEDSUnsafeB();
	bool GetEDSSCCutoff1();
	bool GetEDSSCCutoff2();
	bool GetEDSSCCutoff3();
	bool GetEDSAutoAbortBus();
	bool GetEDSExcessiveRollRateIndication();
	bool GetEDSExcessivePitchYawRateIndication();
	bool GetGuidanceReferenceFailureA();
	bool GetGuidanceReferenceFailureB();
	bool GetFiringCommitInhibit();
	bool GetFiringCommitEnable();
	bool FCCPowerIsOn();
	void SwitchSelector(int stage, int channel);
	bool GetSCCutoffEnabledA();
	bool GetSCCutoffEnabledB();
	bool GetLiftoffEnableA();
	bool GetLiftoffEnableB();
	void GetEDSAbortCommandToSC(bool* abort);
	void AzimuthAlignment(double roll_gimbal);
	bool GetRCA110SyncReset();
	//bool GetRCA110Interrupt();
	void SendLVDCData(int data);
	void SetLVDCPower(bool on);
	int GetLVDAModeRegister();
	void GetLVDAHardwires(int& tag, int& telemetrybuffer);

	void SetIU_ESE(IU_ESE* iu_ese) { ourIU_ESE = iu_ese; };
protected:
	bool GetEDSSCCutoff(int n);
	IU_ESE* ourIU_ESE;
};