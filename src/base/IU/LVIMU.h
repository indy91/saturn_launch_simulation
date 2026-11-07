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

#include "../../util/VectorMath.h"

class IU;

class LVIMU
{
public:
	LVIMU(IU* i);

	void PreStep();
	void PostStep(double simdt);

	VECTOR3 GetAttitude();
	VECTOR3 GetAccumulatedVelocity();

	void DriveGimbals(double x, double y, double z);
	void SetCaged(bool val);
protected:

	void SetAttitudeReference();

	bool Caged;
	MATRIX3 M_NB_REF;	// Current inertial attitude, navigation base to reference
	MATRIX3 M_REF_SM;	// Reference to stable member matrix (REFSMMAT)

	VECTOR3 Attitude;	// RPY
	VECTOR3 AccumulatedVelocity;
	VECTOR3 Measured_Acceleration_T0;

	IU* iu;
};