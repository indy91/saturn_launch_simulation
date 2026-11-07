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

#include "../DelayTimer.h"
#include "../connector.h"
#include "../StageSystems.h"
#include "../SwitchSelector.h"
#include "../Pyro.h"

#define PUVALVE_CLOSED 0
#define PUVALVE_NULL 1
#define PUVALVE_OPEN 2

class BaseSIVBSystems;
class SIVBSystems;

class SIVBSystemsConnector : public Connector
{
public:
	SIVBSystemsConnector();
	virtual ~SIVBSystemsConnector();
	void SetSIVBSystems(SIVBSystems* sivb) { ourSIVB = sivb; };
protected:
	SIVBSystems* ourSIVB;
};

//S-IVB to S-IB or S-II Connector
class SIVBToSIXConnector : public SIVBSystemsConnector
{
public:
	SIVBToSIXConnector();
	virtual ~SIVBToSIXConnector();

	bool HasGround();
	void SISwitchSelector(int channel);
	void SIISwitchSelector(int channel);
	void SetSIThrusterDir(int n, double yaw, double pitch);
	void SetSIIThrusterDir(int n, double yaw, double pitch);

	int GetSwitchSelectorFeedback();
	bool GetLowLevelSensorsDry();
	bool GetSIPropellantDepletionEngineCutoff();
	bool GetSIIPropellantDepletionEngineCutoff();
	void GetSIThrustOK(bool* ok, int n);
	void GetSIIThrustOK(bool* ok);
	void SIEDSCutoff(bool cut);
	void SIIEDSCutoff(bool cut);
	bool GetSIInboardEngineOut();
	bool GetSIOutboardEngineOut();
	double GetSIIFuelTankPressurePSI();
	bool SIXSIVBNotSeparated();
	bool SICSIINotSeparated();
	bool SIIInterstageNotSeparated();
};

class SIVBToIUConnector : public SIVBSystemsConnector
{
public:
	SIVBToIUConnector();
	virtual ~SIVBToIUConnector();

	bool ReceiveMessage(Connector* from, ConnectorMessage& m);
};

//This has class has everything that needs to be copied from Saturn to S-IVB class
class BaseSIVBSystems : public StageSystems
{
public:
	BaseSIVBSystems(Saturn* s);
	virtual ~BaseSIVBSystems();

	void CopyData(BaseSIVBSystems* other);

	void SetVehicleNumber(int VehNo) { VehicleNo = VehNo; }
protected:
	//Forward Power Distributor (411A99) relays
	bool PUInverterAndDCPowerOn; //K1

	//Sequencer Assembly (404A3) relays
	bool LOXTankVentValveOpen; //K1
	bool LOXTankVentAndNPVValvesBoostCloseOn; //K2
	bool LH2TankVentValveOpen; //K3
	bool LH2TankVentAndLatchingReliefValveBoostCloseOn; //K4
	bool LH2TankContinuousVentOrificeSOVOpenOn; //K5
	bool LH2TankContinuousVentValveCloseOn; //K6
	bool LOXTankRepressControlValveOpenOn; //K7
	bool SecondBurnRelay; //K8
	bool FirstBurnRelay; //K9
	bool LH2TankRepressurizationControlValveOpenOn; //K10
	bool LOXTankFlightPressureSystemOn; //K11
	bool LH2TankLatchingReliefValveLatchOn; //K12
	bool LOXTankNPVValveOpenOn; //K13
	bool LOXTankNPVValveLatchOpenOn; //K14
	bool LOXHeatExchangerBypassValveControlEnable; //K15
	bool BurnerExcitersOn; //K16
	bool ChargeUllageIgnitionOn; //K21
	bool ChargeUllageJettisonOn; //K22
	bool FireUllageIgnitionOn; //K23
	bool FireUllageJettisonOn; //K24
	//K29 and K30 are non-latching relays involved in LOX tank pressurization
	//K31 is a non-latching relay indicating LH2 repress control switch enabled
	bool LOXTankPressurizationShutoffValvesClose; //K34
	bool RepressSystemModeSelectOn; //K35
	bool BurnerAutoCutoffSystemArm; //K36
	bool StartTankRechargeValveOpen; //K45
	bool PrevalvesCloseOn; //K46 (K198)
	bool EnginePumpPurgeControlValveEnableOn; //K47
	bool ChilldownShutoffValveCloseOn; //K48
	//K52 is a non-latching relay indicating repress system ambient mode
	bool LH2TankLatchingReliefValveOpenOn; //K53
	bool InflightCalibrateModeOn; //K54
	bool FuelInjTempOKBypass; //K55
	bool LH2TankContinuousVentReliefOverrideSOVOpenOn; //K56
	bool LVDCEngineStopRelay; //K61
	bool PointLevelSensorArmed; //K62
	bool EngineStart; //K63 (K57)
	bool TelemetryCalibrateOn; //K66
	//K71 and K73 are non-latching relays involved in LOX tank pressurization
	//K75 is a non-latching relay indicating LOX repress flight switch enabled
	bool BurnerLOXShutdownValveOpenOn; //K79
	bool BurnerLOXShutdownValveCloseOn; //K80
	bool BurnerLH2PropellantValveOpenOn; //K81
	bool BurnerLH2PropellantValveCloseOn; //K82
	//K87 is a non-latching relay indicating repress system ambient mode
	bool PassivationRelay; //K91
	bool EngineIgnitionPhaseControlValveOpenOn; //K92
	bool StartTankVentControlValveOpenOn; //K93
	bool EngineHeliumControlValveOpen; //K94
	bool EngineMainstageControlValveOpen; //K95
	bool AmbientHeliumSupplyShutoffValveClosedOn; //K96
	bool StartTankRechargeValveArm; //K97

	//Aft Power Distributor Assembly (404A45)
	bool LOXChilldownPumpOn; //K5
	bool FuelChillDownPumpOn; //K6

	// Burner Voter Regulator Module (404A71)
	bool BurnerMalfunctionRelay; // K1

	//J-2 electronics
	bool EngineReady;
	//K11
	bool EngineReadyBypass;

	//K100 and K101
	bool ThrustOKRelay;
	//K105
	bool AftPowerDisableRelay;

	bool RSSEngineStop;
	bool EDSEngineStop;
	bool PropellantDepletionSensors;

	bool EnginePower;
	bool EngineStop;

	bool EDSCutoffDisabled; //K10 and K11
	bool EDSCutoffNo2DisableInhibit; //K23

	double ThrustTimer;
	double ThrustLevel;
	double BoiloffTime;
	double O2H2BurnerTime;
	double GH2_Mass;
	double GO2_Mass;

	//Valve states
	bool LH2CVSOrificeShutoffValve;
	bool LH2CVSReliefOverrideShutoffValve;
	bool LH2LatchingReliefValve;
	bool LH2LatchingReliefValveLatch;
	bool LH2FuelVentAndReliefValve;
	bool HeliumHeaterLH2PropellantValve;
	bool HeliumHeaterLOXShutdownValve;
	bool LOXVentAndReliefValve;
	bool LOXNPVVentAndReliefValve;
	bool LOXNPVValveLatch;

	//Pressure switches
	bool LH2PressureSwitch;
	bool LOXPressureSwitch;

	//Motor switches
	bool EngineControlBusMotorSwitch;

	//Signals
	bool O2H2BurnerOn;
	bool LH2AmbientRepressurizationOn, LH2CryoRepressurizationOn;
	bool LOXAmbientRepressurizationOn, LOXCryoRepressurizationOn;

	bool LH2ReliefValveOpen;
	bool APSUllageOnRelay[2];
	bool AuxHydPumpFlightMode;

	double J2DefaultThrust;
	int PUValveState;
	double MixtureRatio;
	bool propellantInitialized;
	double lastPropellantMass;
	double oxidMass;
	double fuelMass;

	double LH2TankUllagePressurePSI;
	double LOXTankUllagePressurePSI;

	int VehicleNo;

	//Logic
	bool CutoffSignalA, CutoffSignalX;
	bool J2SignalD, J2SignalE, J2SignalF, J2SignalH;
	bool HeliumControlOn;
	bool EMTEMP1, EngineReady1;
	bool StartTankDischargeControlOn;
	bool EngineStartLockUp, SparkSystemOn, EngineStart3, EngineStart4;
	bool SparksDeenergized;
	bool PBSignal1, StartTurbines, PBSignal4;
	bool MainstageSignal, MainstageOn, IgnitionPhaseControlOn;
	bool VCBSignal1, VCBSignal2, VCBSignal3;
	bool IgnitionDetector, CC1Signal1, IgnitionDetectionLockup;
	bool CC2Signal1, CC2Signal2, CC2Signal3, CutoffLockup;
	int EngineState; //0 = Off, 1 = Starting, 2 = Running, 3 = Stopping

	DelayTimer HeliumControlDeenergizedTimer;
	DelayTimer StartTankDischargeDelayTimer;
	DelayTimer IgnitionPhaseTimer;
	DelayTimer SparksDeenergizedTimer;

	Pyro Ullage_Jettison_Pyros;

	bool EngineFailed;
	bool O2H2BurnerFailed;
	//double DebugTimer;
	double F_CVS;
};

class Saturn;
struct ThrustSpec;
struct TankSpec;

class SIVBSystems : public BaseSIVBSystems
{
protected:
	SIVBSystems(Saturn* s, ThrustSpec* j2, TankSpec* j2prop, ThrustSpec* aps, ThrustSpec* ull, ThrustSpec* ver, int num_ver, double PropLoadMR);
public:
	virtual ~SIVBSystems();

	SwitchSelector* GetSwitchSelector() { return &ourSwitchSelector; }

	SIVBToSIXConnector* GetSIVBSIXConnector() { return &sivbSIXConnector; }
	SIVBToIUConnector* GetSIVBIUConnector() { return &sivbIUConnector; }

	void RecalculateEngineParameters(double BaseThrust);
	virtual void RecalculateEngineParameters() = 0;
	virtual void SetSIVBMixtureRatio(double ratio) = 0;
	virtual void SwitchSelectorOutput(int channel) = 0;
	void Timestep(double simdt);
	bool PropellantLowLevel();
	void SetPUValve(int state);

	void EDSEngineCutoff(bool cut) { EDSEngineStop = cut; }
	void SetEngineReadyBypass() { EngineReadyBypass = true; }
	void SetThrusterDir(double beta_y, double beta_p);
	void PointLevelSensorArming() { PointLevelSensorArmed = true; }
	void PointLevelSensorDisarming() { PointLevelSensorArmed = false; }
	void AuxHydPumpFlightModeOn() { AuxHydPumpFlightMode = true; }
	void AuxHydPumpFlightModeOff() { AuxHydPumpFlightMode = false; }
	void APSUllageEngineOn(int n);
	void APSUllageEngineOff(int n);
	void SetAPSAttitudeEngine(int n, bool on);
	void PrevalvesCloseOnReset() { PrevalvesCloseOn = false; }

	bool GetThrustOK() { return ThrustOKRelay; }
	bool Is_Ullage_Jettison_PyrosBlown() { return Ullage_Jettison_Pyros.IsBlown(); }

	void GetJ2ISP(double ratio, double& isp, double& ThrustAdjust);

	//From ESE
	bool ESECommandEngineControlBusPowerOn() { return false; } //TBD
	bool ESECommandEngineControlBusPowerOff() { return false; } //TBD
	//To ESE
	bool BurnerRelaysReset();
	bool RepressRelaysReset();

	//To IU
	double GetLH2TankUllagePressurePSI() { return LH2TankUllagePressurePSI; }
	double GetLOXTankUllagePressurePSI() { return LOXTankUllagePressurePSI; }
	bool GetBurnerMalfunction() { return BurnerMalfunctionRelay; }

	void SetEngineFailed(bool fail);
	void SetO2H2BurnerFailed(bool fail);
protected:

	void UpdatePropellants();
	bool EngineOnLogic();
	void UpdateLH2ValveStates();
	void UpdateLOXValveStates();
	void UpdateO2H2BurnerValveStates();
	void LH2PropellantCalculations(double simdt);
	void LOXPropellantCalculations(double simdt);
	void O2H2Burner(double simdt);
	void ChilldownSystem();
	bool BurnerLogic() const;
	void BurnerShutdown();
	void CheckUllageJettison();

	SIVBToSIXConnector sivbSIXConnector; //Connector to S-IB (Saturn IB) or S-II (Saturn V)
	SIVBToIUConnector sivbIUConnector; //Connector to IU
	ThrustSpec* j2engine;
	ThrustSpec* apsThrusters;
	ThrustSpec* ullage;
	ThrustSpec* vernier;
	int num_vernier;

	TankSpec* main_propellant;

	SwitchSelector ourSwitchSelector;

	//Mixture ratio of propellant load
	const double PropellantLoadMixtureRatio;
};

class SIVB200Systems : public SIVBSystems
{
public:
	SIVB200Systems(Saturn* v, ThrustSpec* j2, TankSpec* j2prop, ThrustSpec* aps, ThrustSpec* ull, ThrustSpec* ver, int num_ver);
	~SIVB200Systems() {};

	void RecalculateEngineParameters();
	void SwitchSelectorOutput(int channel);
	void SetSIVBMixtureRatio(double ratio);
};

class SIVB500Systems : public SIVBSystems
{
public:
	SIVB500Systems(Saturn* v, ThrustSpec* j2, TankSpec* j2prop, ThrustSpec* aps, ThrustSpec* ull, ThrustSpec* ver, int num_ver);
	~SIVB500Systems() {};

	void RecalculateEngineParameters();
	void SwitchSelectorOutput(int channel);
	void SetSIVBMixtureRatio(double ratio);
};