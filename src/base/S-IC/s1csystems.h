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
#include "../Pyro.h"
#include "../StageSystems.h"
#include "../SwitchSelector.h"

class SICSystems;

class SICSystemsConnector : public Connector
{
public:
	SICSystemsConnector();
	virtual ~SICSystemsConnector();
	void SetSICSystems(SICSystems* sic) { ourSIC = sic; };
protected:
	SICSystems* ourSIC;
};

//S-IC to S-II Connector
class SICToSIIConnector : public SICSystemsConnector
{
public:
	SICToSIIConnector();
	virtual ~SICToSIIConnector();

	bool ReceiveMessage(Connector* from, ConnectorMessage& m);
};

//S-IC to S-I Connector
class SICToSIESEConnector : public SICSystemsConnector
{
public:
	SICToSIESEConnector();
	virtual ~SICToSIESEConnector();

	//S-IC to S-I ESE
	bool GetSICThrustOKSimulate(int eng, int n);

	//S-I ESE to S-IC
	bool ReceiveMessage(Connector* from, ConnectorMessage& m);
};

class Saturn;
struct ThrustSpec;

class F1Engine
{
public:
	F1Engine(Saturn* v, ThrustSpec* f1);
	void Timestep(double simdt);

	void SetEngineStart() { EngineStart = true; }
	void SetProgrammedEngineCutoff() { ProgrammedCutoff = true; }
	void SetEDSCutoff() { EDSCutoff = true; }
	void SetGSECutoff() { GSECutoff = true; }
	void SetThrusterDir(double beta_y, double beta_p);
	void SetFailed(bool fail) { EngineFailed = fail; }

	bool GetThrustOK() { return ThrustOK; }
	double GetThrustLevel() { return ThrustLevel; }
	bool GetFailed() { return EngineFailed; }
	bool GetEngineStop() { return EngineStop; }
protected:

	ThrustSpec* th_f1;
	Saturn* vessel;

	bool EngineStart;
	bool EngineStop;
	bool ProgrammedCutoff;
	bool EDSCutoff;
	bool GSECutoff;
	bool RSSCutoff;

	bool ThrustOK;
	bool EngineRunning;

	bool EngineFailed;

	double ThrustTimer;
	double ThrustLevel;

	double pitchPos;
	double yawPos;
};

struct TankSpec;

class SICSystems : public StageSystems
{
public:
	SICSystems(Saturn* s, ThrustSpec* f1, TankSpec* f1prop);
	~SICSystems();
	void Timestep(double misst, double simdt);

	virtual void SetEngineStart(int n);
	void InboardEngineCutoff();
	void OutboardEnginesCutoff();
	void OutboardEnginesCutoffEnable() { PointLevelSensorArmed = true; }
	void OutboardEnginesCutoffDisable() { PointLevelSensorArmed = false; }
	void TwoAdjacentOutboardEnginesOutCutoffEnable() { TwoAdjacentOutboardEnginesOutCutoff = true; }
	void MultipleEngineCutoffEnable() { MultipleEngineCutoffEnabled = true; }
	void EDSEnginesCutoff(bool cut);
	virtual void GSEEnginesCutoff(bool cut);
	void SetThrusterDir(int n, double beta_y, double beta_p);
	void SwitchSelectorOutput(int channel);

	void SetEngineFailed(int n, bool condition);

	bool PropellantLowLevel();
	void GetThrustOK(bool* ok);
	bool GetPropellantDepletionEngineCutoff();
	bool GetInboardEngineOut();
	bool GetOutboardEngineOut();
	bool GetSICSIINotSeparated();

	bool IsPyroBlown();

	virtual bool GetEngineStop();

	SwitchSelector* GetSwitchSelector() { return &ourSwitchSelector; }

	SICToSIIConnector* GetSICToSIIConnector() { return &sicSIIConnector; }
	SICToSIESEConnector* GetSICToSIESEConnector() { return &sicSIESEConnector; }
protected:
	bool TripleVoting(bool vote1, bool vote2, bool vote3);
	double GetSumThrust();

	TankSpec* main_propellant;

	F1Engine f1engine1;
	F1Engine f1engine2;
	F1Engine f1engine3;
	F1Engine f1engine4;
	F1Engine f1engine5;
	F1Engine* f1engines[5];

	Pyro SIC_SII_Separation_Pyros;

	bool PropellantDepletionSensors;
	//K1
	bool TwoAdjacentOutboardEnginesOutCutoff;
	//K2
	bool MultipleEngineCutoffEnabled;
	//K3
	bool PointLevelSensorArmed;

	bool ThrustOK[15];

	SwitchSelector ourSwitchSelector;

	SICToSIIConnector sicSIIConnector;
	SICToSIESEConnector sicSIESEConnector;
};