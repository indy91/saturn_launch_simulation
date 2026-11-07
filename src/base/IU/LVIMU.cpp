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

#include "LVIMU.h"
#include "IU.h"
#include "../Saturn.h"
#include "../OrbMech.h"
#include "../../util/VectorMath.h"

LVIMU::LVIMU(IU* i) : iu(i)
{
	Attitude = AccumulatedVelocity = _V(0, 0, 0);
	Measured_Acceleration_T0 = _V(0, 0, 0);
	M_NB_REF = M_REF_SM = _M(1, 0, 0, 0, 1, 0, 0, 0, 1);
	Caged = true;
}

void LVIMU::PreStep()
{
	iu->GetSat()->GetMeasuredAcceleration(Measured_Acceleration_T0);
	// Convert from reference to stable member coordinates
	Measured_Acceleration_T0 = mul(M_REF_SM, Measured_Acceleration_T0);
}

void LVIMU::PostStep(double simdt)
{
	// MODING
	if (Caged == true && iu->GetControlDistributor()->GetGuidanceReferenceRelease())
	{
		Caged = false;
	}

	// ATTITUDE

	iu->GetSat()->GetRotationMatrix(M_NB_REF);

	if (Caged)
	{
		SetAttitudeReference();
	}
	else
	{
		// Get Euler angles
		MATRIX3 M_NB_SM = mul(M_REF_SM, M_NB_REF);

		OrbMech::DirectionCosineMatrixToEuler(M_NB_SM, Attitude.y, Attitude.z, Attitude.x);
	}

	// ACCELERATION
	VECTOR3 Measured_Acceleration_T1;

	iu->GetSat()->GetMeasuredAcceleration(Measured_Acceleration_T1);

	// Convert from reference to stable member coordinates
	Measured_Acceleration_T1 = mul(M_REF_SM, Measured_Acceleration_T1);

	AccumulatedVelocity += (Measured_Acceleration_T0 + Measured_Acceleration_T1) / 2.0 * simdt;
}

VECTOR3 LVIMU::GetAttitude()
{
	return Attitude;
}

VECTOR3 LVIMU::GetAccumulatedVelocity()
{
	return AccumulatedVelocity;
}

void LVIMU::DriveGimbals(double x, double y, double z)
{
	Attitude.x = x;
	Attitude.y = y;
	Attitude.z = z;

	SetAttitudeReference();
}

void LVIMU::SetCaged(bool val)
{
	Caged = val;
}

void LVIMU::SetAttitudeReference()
{
	MATRIX3 M_NB_SM = OrbMech::EulerToDirectionCosineMatrix(Attitude.y, Attitude.z, Attitude.x);
	M_REF_SM = mul(M_NB_SM, tmat(M_NB_REF));
}