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

#include "Saturn.h"
#include "SaturnLaunchSimulation.h"
#include "../gui/LaunchSimGUIInterface.h"
#include <algorithm>

TankSpec CreatePropellantResource(double mass)
{
	TankSpec tank;

	tank.maxmass = mass;
	tank.mass = mass;

	return tank;
}

ThrustSpec CreateThruster(const VECTOR3& pos, const VECTOR3& dir, double maxth0, TankSpec* ts, double isp0, double area)
{
	ThrustSpec thruster;

	thruster.ref = pos;
	thruster.dir = dir;
	thruster.maxth0 = maxth0;
	thruster.tank = ts;
	thruster.isp0 = isp0;
	thruster.Area = area;
	thruster.level = 0.0;

	return thruster;
}

Vessel::Vessel()
{
	mass = 0.0;
	R = V = Att = AttRate = Measured_Acceleration = _V(0, 0, 0);

	Flin_add = Amom_add = _V(0, 0, 0);
}

Vessel::~Vessel()
{

}

void Vessel::GetAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const
{
	CN = CA = CP = Area = 0.0;
}

void Vessel::SetThrusterLevel(ThrustSpec* ts, double level) const
{
	if (ts->tank && ts->tank->mass)
	{
		ts->level = level;
	}
}

void Vessel::SetThrusterIsp(ThrustSpec* ts, double isp0) const
{
	ts->isp0 = isp0;
}

void Vessel::SetThrusterMax0(ThrustSpec* ts, double maxth0) const
{
	ts->maxth0 = maxth0;
}

void Vessel::SetThrusterDir(ThrustSpec* ts, const VECTOR3& dir) const
{
	ts->dir = unit(dir);
}

void Vessel::SetPropellantMass(TankSpec* ph, double mass) const
{
	ph->mass = mass;
}

void Vessel::SetThrusterResource(ThrustSpec* th, TankSpec* ph) const
{
	th->tank = ph;
	if (!ph) SetThrusterLevel(th, 0);
}

void Vessel::AddForce(const VECTOR3& F, const VECTOR3& r)
{
	VECTOR3 CG = CalculateCG();

	Flin_add += F;

	// Moment vector
	Amom_add = crossp(r - CG, F);
}

double Vessel::GetThrusterLevel(ThrustSpec* th) const
{
	return th->level;
}

double Vessel::GetPropellantMass(TankSpec* ph) const
{
	return ph->mass;
}

double Vessel::GetPropellantMaxMass(TankSpec* ph) const
{
	return ph->maxmass;
}

void Vessel::GetAngularVel(VECTOR3& rates) const
{
	rates = AttRate;
}

void Vessel::GetAttitude(VECTOR3& att) const
{
	att = Att;
}

void Vessel::GetMeasuredAcceleration(VECTOR3& accel) const
{
	accel = Measured_Acceleration;
}

void Vessel::GetRotationMatrix(MATRIX3& R) const
{
	R = R0;
}

void Vessel::UpdateState(const VECTOR3& r, const VECTOR3& v, const VECTOR3& att, const VECTOR3& attrate, const VECTOR3& measured_accel)
{
	R = r;
	V = v;
	Att = att;
	AttRate = attrate;
	Measured_Acceleration = measured_accel;

	// Calculate rotation matrix
	R0 = OrbMech::EulerToDirectionCosineMatrix(Att.x, Att.y, Att.z);
}

void Vessel::ResetAdditionalForces()
{
	Flin_add = Amom_add = _V(0, 0, 0);
}

void Vessel::UpdateThruster(ThrustSpec* thruster, double simdt) const
{
	TankSpec* ts;
	double th;

	if (thruster->level = std::max(0.0, std::min(1.0, thruster->level)))
	{
		if ((ts = thruster->tank) && ts->mass) // fuel available?
		{
			th = thruster->maxth0 * thruster->level;
			ts->mass -= th / thruster->isp0 * simdt;
			if (ts->mass < 0.0) ts->mass = 0.0;
		}
		else thruster->level = 0.0; // No fuel
	}
}

void Vessel::ThrusterForceMoment(const ThrustSpec* thruster, const VECTOR3& CG, double press, VECTOR3& F, VECTOR3& M) const
{
	// Calculate force
	double f;

	f = thruster->maxth0 - thruster->Area * press;
	if (f < 0.0) f = 0.0;
	f *= thruster->level;

	// Force vector in body coordinates
	F = thruster->dir * f;

	// Moment vector
	M = crossp(thruster->ref - CG, F);
}

Saturn::Saturn() : ml(this)
{
	SI_EmptyMass = 0.0;
	S4B_EmptyMass = 0.0;
	S4PL_Mass = 0.0;
	LET_Mass = 0.0;
	Interstage_Mass = 0.0;
	S4B_UllageCaseEmptyMass = 0.0;
	GMTLO = 0.0;

	stage = 0;
	SI_RetroNum = 0;
	S4B_UllageNum = 0;

	bInterstageAttached = false;
	bLETAttached = false;
	bSIVBUllageCasingsAttached = false;

	T = 0.0;

	gui_interface = NULL;
}

Saturn::~Saturn()
{

}

int Saturn::Init(const SaturnLaunchSimulationInputs* in)
{
	gui_interface = in->gui_interface;

	GMTLO = in->t_Launch;

	SI_EmptyMass = in->SI_EmptyMass;
	S4B_EmptyMass = in->S4B_EmptyMass;
	S4PL_Mass = in->S4PL_Mass;
	LET_Mass = in->LET_Mass;
	Interstage_Mass = in->SII_InterstageMass;
	SI_RetroNum = in->SI_RetroNum;
	S4B_UllageNum = in->S4B_UllageNum;
	S4B_UllageCaseEmptyMass = in->S4B_UllageCaseEmptyMass;

	// Create propellants
	SIMainPropellant = CreatePropellantResource(in->SI_MainFuelMass);
	for (int i = 0; i < in->SI_RetroNum; i++)
	{
		SIRetroRocketPropellant[i] = CreatePropellantResource(in->SI_RetroFuelMass);
	}
	SIVBMainPropellant = CreatePropellantResource(in->S4B_MainFuelMass);
	for (int i = 0; i < in->S4B_UllageNum; i++)
	{
		SIVBUllageRocketPropellant[i] = CreatePropellantResource(in->S4B_UllageFuelMass);
	}
	SIVBAPSPropellant[0] = CreatePropellantResource(in->S4B_APSPropellant);
	SIVBAPSPropellant[1] = CreatePropellantResource(in->S4B_APSPropellant);

	// Create thrusters
	double d = 128.69 * 0.0254; // Distances from centerline to engine position
	double a = 9.7269;
	double cl = 100.0 * 0.0254;

	SIEngines[0] = CreateThruster(_V(cl, -d, d), _V(1.0, 0.0, 0.0), in->THRUST_FIRST_VAC, &SIMainPropellant, in->ISP_FIRST_VAC, a);
	SIEngines[1] = CreateThruster(_V(cl, -d, -d), _V(1.0, 0.0, 0.0), in->THRUST_FIRST_VAC, &SIMainPropellant, in->ISP_FIRST_VAC, a);
	SIEngines[2] = CreateThruster(_V(cl, d, -d), _V(1.0, 0.0, 0.0), in->THRUST_FIRST_VAC, &SIMainPropellant, in->ISP_FIRST_VAC, a);
	SIEngines[3] = CreateThruster(_V(cl, d, d), _V(1.0, 0.0, 0.0), in->THRUST_FIRST_VAC, &SIMainPropellant, in->ISP_FIRST_VAC, a);
	SIEngines[4] = CreateThruster(_V(cl, 0.0, 0.0), _V(1.0, 0.0, 0.0), in->THRUST_FIRST_VAC, &SIMainPropellant, in->ISP_FIRST_VAC, a);

	for (int i = 0; i < SI_RetroNum; i++)
	{
		SIRetroRockets[i] = CreateThruster(_V(0, 0, 0), _V(1.0, 0.0, 0.0), 75800.0 * 4.4482216152605, &SIRetroRocketPropellant[i], 2300.0, 0.1);
	}

	// Initialize state
	stage = 0;
	bInterstageAttached = true;
	bLETAttached = true;
	bSIVBUllageCasingsAttached = true;

	ml.Init(GMTLO, in->IsSaturnV);

	return 0;
}

void Saturn::PreStep(double t, double dt)
{
	T = t;

	if (stage == PRELAUNCH_STAGE) ml.PreStep(t, dt);
}

void Saturn::PostStep(double t, double dt)
{
	T = t;
}

void Saturn::UpdatePropellants(double simdt)
{
	UpdateThruster(&SIVBJ2Engine, simdt);
	for (int i = 0; i < S4B_UllageNum; i++)
	{
		UpdateThruster(&SIVBUllageRockets[i], simdt);
	}
	for (int i = 0; i < 6; i++)
	{
		UpdateThruster(&SIVBAPSEngines[i], simdt);
	}
	UpdateThruster(&SIVBAPSUllageEngines[0], simdt);
	UpdateThruster(&SIVBAPSUllageEngines[1], simdt);
}

int Saturn::GetStage()
{
	return stage;
}

void Saturn::SetStage(int s)
{
	stage = s;
	UpdateMass();
}

void Saturn::SetThrusterDir(ThrustSpec* th, double beta_y, double beta_p) const
{
	// Applicable to main engines gimballing

	VECTOR3 dir;
	double tan_beta_p_eff, tan_beta_y_eff, K;

	tan_beta_p_eff = tan(beta_p);
	tan_beta_y_eff = tan(beta_y);

	K = sqrt(1.0 + pow(tan_beta_p_eff, 2) + pow(tan_beta_y_eff, 2));

	dir = _V(1.0, tan_beta_y_eff, -tan_beta_p_eff) / K;

	Vessel::SetThrusterDir(th, dir);
}

void Saturn::SendEventMessage(const std::string& message)
{
	gui_interface->SendEventMessage(T - GMTLO, message);
}

void Saturn::SendTelemetryMessage(const std::string& message)
{
	gui_interface->SendTelemetryMessage(T - GMTLO, message);
}

int Saturn::GetControl(int num)
{
	return gui_interface->GetControl(num);
}