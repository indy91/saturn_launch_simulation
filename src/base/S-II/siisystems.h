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

#include "../j2engine.h"
#include "../connector.h"
#include "../Pyro.h"
#include "../SwitchSelector.h"
#include "../StageSystems.h"

#define PUVALVE_CLOSED 0
#define PUVALVE_NULL 1
#define PUVALVE_OPEN 2

class SIISystems;

class SIISystemsConnector : public Connector
{
public:
	SIISystemsConnector();
	virtual ~SIISystemsConnector();
	void SetSIISystems(SIISystems* sii) { ourSII = sii; };
protected:
	SIISystems* ourSII;
};

//S-II to S-IVB Connector
class SIIToSIVBConnector : public SIISystemsConnector
{
public:
	SIIToSIVBConnector();
	virtual ~SIIToSIVBConnector();

	bool ReceiveMessage(Connector* from, ConnectorMessage& m);
};

//S-II to S-II Interstage Connector
class SIIToSIIInterstageConnector : public SIISystemsConnector
{
public:
	SIIToSIIInterstageConnector();
	virtual ~SIIToSIIInterstageConnector();

	void SetSIThrusterDir(int n, double yaw, double pitch);
	void SIEDSCutoff(bool cut);
	void SISwitchSelector(int channel);
	int GetSwitchSelectorFeedback();
	void GetSIThrustOK(bool* ok, int n);
	bool GetSIPropellantDepletionEngineCutoff();
	bool GetSIInboardEngineOut();
	bool GetSIOutboardEngineOut();
	bool GetSICSIINotSeparated();
};

class Saturn;
struct TankSpec;

class SIISystems : public StageSystems
{
public:
	SIISystems(Saturn* s, ThrustSpec* j2, TankSpec* j2prop, ThrustSpec* ull, int ull_num);
	virtual ~SIISystems();
	void Timestep(double simdt);

	void LVDCEnginesCutoff();
	void LVDCCenterEngineCutoff();
	void LVDCEnginesCutoffReset();
	void SetEnginesReadyBypass();
	void ResetEnginesReadyBypass();
	void EngineStartOn();
	void EngineStartOff();
	void EDSEnginesCutoff(bool cut);
	void SetDepletionSensorsCutoffArm() { PointLevelSensorArmed = true; }
	void SetDepletionSensorsCutoffDisarm() { PointLevelSensorArmed = false; }
	void FireUllageTrigger() { UllageTrigger = true; }
	void SetOrdnanceArm() { OrdnanceArmed = true; }
	void SetSIISIVBOrdnanceArm() { SIISIVBOrdnanceArmed = true; }
	void SetThrusterDir(int n, double beta_y, double beta_p);
	void SetPUValve(int state);
	void StartPhaseLimiterCutoffArmReset() { StartPhaseLimiterCutoffArm = false; }
	void LH2StepPressurizationReset() { LH2StepPressurization = false; }

	void RecalculateEngineParameters();
	void RecalculateEngineParameters(double BaseThrust);
	void SetMixtureRatio(double ratio);
	void SwitchSelectorOutput(int channel);

	void SetEngineFailed(int n, bool fail);

	void GetThrustOK(bool* ok);
	bool GetPropellantDepletionEngineCutoff();

	//To IU
	double GetLH2TankUllagePressurePSI() { return LH2TankUllagePressurePSI; }
	bool GetSIISIVBNotSeparated();

	SIIToSIVBConnector* GetSIIToSIVBConnector() { return &siiSIVBConnector; }
	SIIToSIIInterstageConnector* GetSIIToSIIInterstageConnector() { return &siiSIIInterstageConnector; }

	SwitchSelector* GetSwitchSelector() { return &ourSwitchSelector; }

	bool IsPyroBlown() { return SII_SIVB_Separation_Pyros.IsBlown(); }

	Pyro SII_Interstage_Pyros;
	Pyro SII_SIVB_Separation_Pyros;
protected:
	void GetJ2ISP(double ratio, double& isp, double& ThrustAdjust);
	bool PropellantLowLevel();

	bool ThrustOK[5];

	int PUValveState;

	//K20
	bool StartPhaseLimiterCutoffArm;
	//K18 (LOX), K23 (LH2)
	bool PointLevelSensorArmed;
	//K16 (K37)
	bool SICSIIEngineStartLockout;
	//K17 (K43)
	bool CommandEngineStart;
	//K50
	bool LH2StepPressurization;
	bool PropellantDepletionSensors;
	bool UllageTrigger;
	bool OrdnanceArmed;
	bool SIISIVBOrdnanceArmed;

	double J2DefaultThrust;

	double LH2TankUllagePressurePSI;

	J2Engine j2engine1;
	J2Engine j2engine2;
	J2Engine j2engine3;
	J2Engine j2engine4;
	J2Engine j2engine5;
	J2Engine* j2engines[5];
	ThrustSpec* ullage;
	int ullage_num;
	TankSpec* main_propellant;

	SwitchSelector ourSwitchSelector;

	SIIToSIVBConnector siiSIVBConnector;
	SIIToSIIInterstageConnector siiSIIInterstageConnector;
};

// INTERSTAGE, MOVE THIS LATER TO DIFFERENT FILES

class SIIInterstageSystems;

//Generic S-II interstage connector class
class SIIInterstageSystemsConnector : public Connector
{
public:
	SIIInterstageSystemsConnector();
	virtual ~SIIInterstageSystemsConnector();
	void SetSIIInterstageSystems(SIIInterstageSystems* is) { ourIS = is; };
protected:
	SIIInterstageSystems* ourIS;
};

//Connector from S-II interstage to S-II
class SIIInterstageToSIIConnector : public SIIInterstageSystemsConnector
{
public:
	SIIInterstageToSIIConnector();
	virtual ~SIIInterstageToSIIConnector();

	bool ReceiveMessage(Connector* from, ConnectorMessage& m);
};

//Connector for S-II interstage to S-IC
class SIIInterstageToSICConnector : public SIIInterstageSystemsConnector
{
public:
	SIIInterstageToSICConnector();
	virtual ~SIIInterstageToSICConnector();

	bool ForwardMessage(ConnectorMessage& m);
};

class SIIInterstageSystems
{
public:
	SIIInterstageSystems();
	virtual ~SIIInterstageSystems();

	SIIInterstageToSIIConnector* GetSIIInterstageToSIIConnector() { return &siiInterstageSIIConnector; }
	SIIInterstageToSICConnector* GetSIIInterstageToSICConnector() { return &siiInterstageSICConnector; }
protected:
	SIIInterstageToSIIConnector siiInterstageSIIConnector;
	SIIInterstageToSICConnector siiInterstageSICConnector;
};