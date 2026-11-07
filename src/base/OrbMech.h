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

namespace OrbMech
{
	struct GlobalConstants
	{
		GlobalConstants();

		double R_E;		// Equatorial radius, meters
		double ELLIPT;	// Ellipticity constant
		double w_E;		// Rotation rate, rad/sec
		double J2;
		double J3;
		double J4;
		double MU;
	};

	struct SessionConstants
	{

	};

	// CONVERSIONS
	// Geodetic to Earth-fixed vector
	VECTOR3 GEODETIC_TO_EF(double LAT_GEOD, double LON, double ALT, double ELLIPT, double EARTH_RADIUS_EQUATOR);
	// Earth-fixed vector to geodetic
	void EF_TO_GEODETIC(const VECTOR3& R_EF, double& LAT_GEOD, double& LON, double& ALT, double ELLIPT, double EARTH_RADIUS_EQUATOR);
	// Radius of ellipsoid
	double RadiusEllipsoid(VECTOR3 R, double EARTH_RADIUS_EQUATOR, double ELLIPT);
	double RadiusEllipsoid(double SINL, double EARTH_RADIUS_EQUATOR, double ELLIPT);
	// Polar equatorial to plumbline coordinates
	MATRIX3 EquatorialToPlumbline(double LAT_GEOD, double LNG, double AZI);
	// Body to inertial
	MATRIX3 EulerToDirectionCosineMatrix(double theta, double psi, double phi);
	// Inertial to body
	void DirectionCosineMatrixToEuler(const MATRIX3& M, double& theta, double& psi, double& phi);
	// Inertial to Earth-fixed
	MATRIX3 InertialEquatorialToEarthFixed(double t, double w_E);

	// Moments to angular acceleration
	VECTOR3 EulersEquationInv(VECTOR3 I, VECTOR3 M, VECTOR3 W);
	// Body rates to Euler rates
	VECTOR3 BodyToEulerRates(VECTOR3 w_B, VECTOR3 Att);

	// ATMOSPHERE
	void PRA63(double Z, double& press, double& dens, double &spos);

	// ANGLES
	void normalizeAngle(double& ang);
	void normalizeAngle(VECTOR3& ang);
	double acos2(double _X);
};