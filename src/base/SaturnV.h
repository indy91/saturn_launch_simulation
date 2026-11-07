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

#include "Saturn.h"

struct SaturnLaunchSimulationInputs;
class SICSystems;
class SIIInterstageSystems;
class SIISystems;
class SIVB500Systems;
class IU;
class IUSV;

class SaturnV : public Saturn
{
public:
	SaturnV();
	virtual ~SaturnV();

	int Init(const SaturnLaunchSimulationInputs* in);

	void PreStep(double t, double dt);
	void PostStep(double t, double dt);

	void UpdatePropellants(double simdt);
	void UpdateThrustForces(const VECTOR3& CG, double press, VECTOR3& F_sum, VECTOR3& M_sum) const;
	void UpdateMass();
	VECTOR3 CalculateCG() const;
	VECTOR3 CalculatePMI() const;
	void GetAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const;

	IU* GetIU();

protected:
	void GetSICAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const;
	void GetSIIAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const;
	void GetSIVBAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const;
private:
	// EMPTY MASS
	double SII_EmptyMass;

	// PROPERTIES
	int SII_UllageNum;
	int SII_RetroNum;

	// PROPELLANTS
	TankSpec SIIMainPropellant;
	TankSpec SIIUllageRocketPropellant[8];
	TankSpec SIIRetroRocketPropellant[8];

	// THRUSTERS
	ThrustSpec SIIUllageRockets[8];
	ThrustSpec SIIJ2Engines[5];
	ThrustSpec SIIRetroRockets[8];

	// SYSTEMS
	SICSystems* SIC;
	SIIInterstageSystems* SII_IS;
	SIISystems* SII;
	SIVB500Systems* SIVB;
	IUSV* IU;
};