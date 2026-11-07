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

#include "../util/VectorMath.h"
#include "../gui/LaunchSimGUIInterface.h"
#include "GSE/ML.h"

struct SaturnLaunchSimulationInputs;

struct TankSpec
{
	double maxmass; // max propellant mass
	double mass;	// current propellant mass
};

struct ThrustSpec
{
	VECTOR3 ref;
	VECTOR3 dir;
	double maxth0;
	double isp0;
	double Area;
	double level;
	TankSpec* tank;	// Propellant resource for this thruster
};

TankSpec CreatePropellantResource(double mass);
ThrustSpec CreateThruster(const VECTOR3& pos, const VECTOR3& dir, double maxth0, TankSpec* ts, double isp0, double area);

class Vessel
{
public:
	Vessel();
	virtual ~Vessel();

	virtual void PreStep(double t, double dt) {}
	virtual void PostStep(double t, double dt) {}

	virtual void UpdatePropellants(double simdt) = 0;
	virtual void UpdateThrustForces(const VECTOR3& CG, double press, VECTOR3& F_sum, VECTOR3& M_sum) const = 0;
	virtual void UpdateMass() = 0;
	virtual VECTOR3 CalculateCG() const = 0;

	virtual void GetAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const;

	double GetMass() const { return mass; }

	void SetThrusterLevel(ThrustSpec* ts, double level) const;
	void SetThrusterIsp(ThrustSpec* ts, double isp0) const;
	void SetThrusterMax0(ThrustSpec* ts, double maxth0) const;
	void SetThrusterDir(ThrustSpec* th, const VECTOR3& dir) const;
	void SetPropellantMass(TankSpec* ph, double mass) const;
	void SetThrusterResource(ThrustSpec* th, TankSpec* ph) const;

	void AddForce(const VECTOR3& F, const VECTOR3& r);

	double GetThrusterLevel(ThrustSpec* th) const;
	double GetPropellantMass(TankSpec* ph) const;
	double GetPropellantMaxMass(TankSpec* ph) const;
	void GetAngularVel(VECTOR3& rates) const;
	void GetAttitude(VECTOR3& att) const;
	void GetMeasuredAcceleration(VECTOR3& accel) const;
	void GetRotationMatrix(MATRIX3& R) const;

	void UpdateState(const VECTOR3& r, const VECTOR3& v, const VECTOR3& att, const VECTOR3& attrate, const VECTOR3& measured_accel);
	void ResetAdditionalForces();
protected:
	void SetMass(double m) { mass = m; }

	void UpdateThruster(ThrustSpec* thruster, double simdt) const;
	void ThrusterForceMoment(const ThrustSpec* thruster, const VECTOR3& CG, double press, VECTOR3& F, VECTOR3& M) const;

	// Additional forces
	VECTOR3 Flin_add;
	VECTOR3 Amom_add;
private:
	// STATE
	VECTOR3 R;
	VECTOR3 V;
	VECTOR3 Att;
	VECTOR3 AttRate;
	VECTOR3 Measured_Acceleration;
	MATRIX3 R0; // Body to inertial coordinates

	// Total mass
	double mass;
};

#define PRELAUNCH_STAGE		0
#define LAUNCH_STAGE_ONE	1
#define LAUNCH_STAGE_TWO	2
#define LAUNCH_STAGE_SIVB	3

class IU;

class Saturn : public Vessel
{
public:
	Saturn();
	virtual ~Saturn();

	virtual int Init(const SaturnLaunchSimulationInputs* in);

	virtual void PreStep(double t, double dt);
	virtual void PostStep(double t, double dt);

	virtual void UpdatePropellants(double simdt);

	int GetStage();
	void SetStage(int s);
	virtual IU* GetIU() = 0;

	virtual VECTOR3 CalculatePMI() const = 0;

	void SetThrusterDir(ThrustSpec* th, double beta_y, double beta_p) const;

	void SendEventMessage(const std::string& message);
	void SendTelemetryMessage(const std::string& message);
	int GetControl(int num);

	ML* GetML() { return &ml; }
protected:

	// EMPTY MASS
	double SI_EmptyMass;
	double S4B_EmptyMass;
	double S4PL_Mass;
	double LET_Mass;
	double Interstage_Mass;
	double S4B_UllageCaseEmptyMass;

	// PROPERTIES
	int SI_RetroNum;
	int S4B_UllageNum;
	double GMTLO;

	// STATE
	int stage;
	bool bInterstageAttached;
	bool bLETAttached;
	bool bSIVBUllageCasingsAttached;
	double T;

	// PROPELLANTS
	// S-I
	TankSpec SIMainPropellant;
	TankSpec SIRetroRocketPropellant[8];
	// S-IVB
	TankSpec SIVBMainPropellant;
	TankSpec SIVBUllageRocketPropellant[4];
	TankSpec SIVBAPSPropellant[2];

	// THRUSTERS
	ThrustSpec SIEngines[5];
	ThrustSpec SIRetroRockets[8];
	ThrustSpec SIVBUllageRockets[4];
	ThrustSpec SIVBJ2Engine;
	ThrustSpec SIVBAPSEngines[6];
	ThrustSpec SIVBAPSUllageEngines[2];

	// STAGE SYSTEMS INCLUDING GSE
	ML ml;

	// POINTERS
	LaunchSimGUIInterface* gui_interface;
};