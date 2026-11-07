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
#include "eds.h"
#include "FCC.h"
#include "IUControlDistributor.h"
#include "../DelayTimer.h"
#include "IUAuxiliaryPowerDistributor.h"
#include "LVDA.h"
#include "IUControlSignalProcessor.h"
#include "LVRG.h"
#include "LVIMU.h"
#include "LVDC.h"
#include "../StageSystems.h"
#include "../SwitchSelector.h"

class IU;

//IU connector class

class IUConnector : public Connector
{
public:
	IUConnector();
	virtual ~IUConnector();

	void SetIU(IU* iu) { ourIU = iu; };
protected:
	IU* ourIU;
};

// IU to S-IVB command connector
class IUToSIVBCommandConnector : public IUConnector
{
public:
	IUToSIVBCommandConnector();
	~IUToSIVBCommandConnector();

	void SetAPSAttitudeEngine(int n, bool on);
	void SIEDSCutoff(bool cut);
	void SIIEDSCutoff(bool cut);
	void SIVBEDSCutoff(bool cut);
	void SetSIThrusterDir(int n, double yaw, double pitch);
	void SetSIIThrusterDir(int n, double yaw, double pitch);
	void SetSIVBThrusterDir(double yaw, double pitch);
	void SISwitchSelector(int channel);
	void SIISwitchSelector(int channel);
	void SIVBSwitchSelector(int channel);
	int GetSwitchSelectorFeedback();
	void GetSIThrustOK(bool* ok, int n);
	void GetSIIThrustOK(bool* ok);
	bool GetSIVBThrustOK();
	bool GetSIPropellantDepletionEngineCutoff();
	bool GetSIIPropellantDepletionEngineCutoff();
	bool GetSIBLowLevelSensorsDry();
	bool GetSIInboardEngineOut();
	bool GetSIOutboardEngineOut();
	double GetSIIFuelTankPressurePSI();
	double GetSIVBLOXTankPressurePSI();
	double GetSIVBFuelTankPressurePSI();
	bool SIXSIVBNotSeparated();
	bool SICSIINotSeparated();
	bool SIIInterstageNotSeparated();
	bool GetSIVBBurnerMalfunction();
};

//IU to IU ESE command connector
class IUToIUESECommandConnector : public IUConnector
{
public:
	IUToIUESECommandConnector();
	~IUToIUESECommandConnector();

	bool ReceiveMessage(Connector* from, ConnectorMessage& m);

	bool IsUmbilicalConnected();

	bool GetCommandVehicleLiftoffIndicationInhibit();
	bool GetExcessiveRollRateAutoAbortInhibit(int n);
	bool GetExcessivePitchYawRateAutoAbortInhibit(int n);
	bool GetTwoEngineOutAutoAbortInhibit(int n);
	bool GetGSEOverrateSimulate(int n);
	bool GetEDSPowerInhibit();
	bool PadAbortRequest();
	bool GetEngineThrustIndicationEnableInhibitA();
	bool GetEngineThrustIndicationEnableInhibitB();
	bool EDSLiftoffInhibitA();
	bool EDSLiftoffInhibitB();
	bool GetSIBurnModeSubstitute();
	bool GetGuidanceReferenceRelease();
	bool GetGuidanceReferenceReleaseAlert();
	bool GetQBallSimulateCmd();
	bool GetEDSAutoAbortSimulate(int n);
	bool GetEDSLVCutoffSimulate(int n);
	bool GetSICOutboardEnginesCantInhibit();
	bool GetSICOutboardEnginesCantSimulate();
	bool GetRCA110Interrupt();
	bool GetRCA110Sync();
	void LVDC_To_RCA110_Interrupt(bool set);
};

class IUToCSMCommandConnector
{
public:
	IUToCSMCommandConnector(IU* iu);
	~IUToCSMCommandConnector();

	bool GetSIISIVbDirectStagingSignal();
	bool GetTLIInhibitSignal();
	bool GetCMCSIVBTakeover();
	bool GetSIVBInjSeqStart();
	bool GetSIVBCutoff();
protected:
	IU* ourIU;
};

class Saturn;

class IU : public StageSystems
{

public:
	IU(Saturn* s);
	virtual ~IU();

	int SetMissionInfo(bool crewed, bool sccontpowered, const std::string& LVDCSoftwareName);

	virtual void ConnectToSIVB(Connector* CommandConnector);

	void DisconnectIU();

	///
	/// \brief Timestep function.
	/// \param simt The current Mission Elapsed Time in seconds from launch.
	/// \param simdt The time in seconds since the last timestep call.
	///
	virtual void PreStep(double simt, double simdt);
	void PostStep(double simt, double simdt);

	virtual void SwitchSelectorOutput(int item) = 0;

	bool GetSIPropellantDepletionEngineCutoff();
	virtual bool SIBLowLevelSensorsDry();
	virtual bool GetSIIPropellantDepletionEngineCutoff();
	bool GetSIInboardEngineOut();
	bool GetSIOutboardEngineOut();
	virtual bool GetSIIInboardEngineOut() { return false; }
	virtual bool GetSIIOutboardEngineOut() { return false; }
	virtual bool GetSIIEnginesOut();
	bool GetSCControlPoweredFlight() { return SCControlPoweredFlight; }

	IUToCSMCommandConnector* GetCommandConnector() { return &commandConnector; }
	IUToSIVBCommandConnector* GetSIVBCommandConnector() { return &sivbCommandConnector; }
	IUToIUESECommandConnector* GetIUToIUESECommandConnector() { return &iuToIUESECommandConnector; }

	Saturn* GetSat() { return sat; }

	//Subsystem Access
	virtual EDS* GetEDS() = 0;
	virtual FCC* GetFCC() = 0;
	virtual IUControlDistributor* GetControlDistributor() = 0;
	virtual DelayTimer* GetEngineCutoffEnableTimer() = 0;
	virtual LVDC* GetLVDC() = 0;
	LVRG* GetLVRG() { return &lvrg; }
	virtual LVDA* GetLVDA() = 0;
	LVIMU* GetLVIMU() { return &lvimu; }
	IUAuxiliaryPowerDistributor2* GetAuxPowrDistr() { return &AuxiliaryPowerDistributor2; }
	IUControlSignalProcessor* GetContSigProc() { return &ControlSignalProcessor; }
	SwitchSelector* GetSwitchSelector() { return &ourSwitchSelector; }
protected:

	int State;

	//
	// Saturn stuff
	//

	bool Crewed;
	bool SCControlPoweredFlight;

	//Time keeping for LVDC
	double LastCycled;

	// Connector to CSM
	IUToCSMCommandConnector commandConnector;
	//Connector to S-IVB
	IUToSIVBCommandConnector sivbCommandConnector;
	//Connector to IU UESE
	IUToIUESECommandConnector iuToIUESECommandConnector;

	//Subsystems:

	//603A19
	LVIMU lvimu;
	//602A23
	LVRG lvrg;
	//601A33
	IUAuxiliaryPowerDistributor1 AuxiliaryPowerDistributor1;
	//602A34
	IUAuxiliaryPowerDistributor2 AuxiliaryPowerDistributor2;
	//601A24
	IUControlSignalProcessor ControlSignalProcessor;
	SwitchSelector ourSwitchSelector;
};

class IU1B :public IU
{
public:
	IU1B(Saturn* s);
	~IU1B();
	void PreStep(double simt, double simdt);
	bool SIBLowLevelSensorsDry();
	void SwitchSelectorOutput(int item);

	//Subsystem access
	FCC* GetFCC() { return &fcc; }
	EDS* GetEDS() { return &eds; }
	IUControlDistributor* GetControlDistributor() { return &ControlDistributor; }
	DelayTimer* GetEngineCutoffEnableTimer() { return &EngineCutoffEnableTimer; }
	LVDC* GetLVDC() { return &lvdc; }
	LVDA* GetLVDA() { return &lvda; }

protected:
	//603A29
	LVDA1B lvda;
	//603A28
	LVDC1B lvdc;
	//602A27
	FCC1B fcc;
	//602A5
	EDS1B eds;
	IUControlDistributor1B ControlDistributor;
	//603A36
	DelayTimer EngineCutoffEnableTimer;
};

class IUSV :public IU
{
public:
	IUSV(Saturn* s);
	~IUSV();
	void PreStep(double simt, double simdt);
	bool GetSIIPropellantDepletionEngineCutoff();
	bool GetSIIEnginesOut();
	bool GetSIIInboardEngineOut();
	bool GetSIIOutboardEngineOut();
	void SwitchSelectorOutput(int item);

	//Subsystem access
	FCC* GetFCC() { return &fcc; }
	EDS* GetEDS() { return &eds; }
	IUControlDistributor* GetControlDistributor() { return &ControlDistributor; }
	DelayTimer* GetEngineCutoffEnableTimer() { return &EngineCutoffEnableTimer; }
	LVDC* GetLVDC() { return &lvdc; }
	LVDA* GetLVDA() { return &lvda; }

protected:
	//603A29
	LVDASV lvda;
	//603A28
	LVDCSV lvdc;
	//602A27
	FCCSV fcc;
	//602A5
	EDSSV eds;
	//603A2
	IUControlDistributorSV ControlDistributor;
	//603A36
	DelayTimer EngineCutoffEnableTimer;
};