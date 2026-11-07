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

#include "OrbMech.h"
#include "../util/Constants.h"

namespace OrbMech
{
	GlobalConstants::GlobalConstants()
	{
		R_E = 6378165.0;
		ELLIPT = 0.00335232997;
		w_E = 7.292107787879948e-05;
		J2 = 1.0823e-3;
		J3 = -2.51e-6;
		J4 = -1.60e-6;
		MU = 3.986032e14;
	}

	VECTOR3 GEODETIC_TO_EF(double LAT_GEOD, double LON, double ALT, double ELLIPT, double EARTH_RADIUS_EQUATOR)
	{
		// LAT_GEOD: Geodetic latitude, rad
		// LON: Longitude, rad
		// ALT: Altitude, m
		// ELLIPT: Ellipticity constant
		// EARTH_RADIUS_EQUATOR: Equatorial radius, m

		double CLAT, SLAT, DUM, DUM1, R_EF_EQUAT;

		CLAT = cos(LAT_GEOD);
		SLAT = sin(LAT_GEOD);
		DUM = pow(1.0 - ELLIPT, 2);
		DUM1 = EARTH_RADIUS_EQUATOR / sqrt(CLAT * CLAT + SLAT * SLAT * DUM);
		R_EF_EQUAT = (DUM1 + ALT) * CLAT;

		return _V(R_EF_EQUAT * cos(LON), R_EF_EQUAT * sin(LON), (DUM * DUM1 + ALT) * SLAT);
	}

	void EF_TO_GEODETIC(const VECTOR3& R_EF, double& LAT_GEOD, double& LON, double& ALT, double ELLIPT, double EARTH_RADIUS_EQUATOR)
	{
		double FLATCON, A_O, R_XY, R, SIN_P, COS_P, RAD_P, DEL, DEL_LAT, PHI;

		FLATCON = 1.0 - pow(1.0 - ELLIPT, 2);
		A_O = 1.0 - FLATCON;
		R_XY = R_EF.x * R_EF.x + R_EF.y * R_EF.y;
		R = sqrt(R_XY + R_EF.z * R_EF.z);
		SIN_P = R_EF.z / R;
		COS_P = sqrt(R_XY) / R;
		RAD_P = EARTH_RADIUS_EQUATOR / sqrt(1.0 + FLATCON * SIN_P * SIN_P / A_O);
		DEL = (FLATCON * SIN_P * COS_P) / (1.0 - FLATCON * COS_P * COS_P);
		DEL_LAT = RAD_P * DEL / R;
		PHI = atan(SIN_P / COS_P);
		LAT_GEOD = PHI + DEL_LAT;
		LON = atan2(R_EF.y, R_EF.x);
		ALT = (R - RAD_P) * (1.0 - 0.5 * DEL * DEL_LAT);
	}

	double RadiusEllipsoid(VECTOR3 R, double EARTH_RADIUS_EQUATOR, double ELLIPT)
	{
		double SINL;

		SINL = dotp(unit(R), _V(0.0, 0.0, 1.0));

		return RadiusEllipsoid(SINL, EARTH_RADIUS_EQUATOR, ELLIPT);
	}

	double RadiusEllipsoid(double SINL, double EARTH_RADIUS_EQUATOR, double ELLIPT)
	{
		return (1.0 - ELLIPT) * EARTH_RADIUS_EQUATOR / sqrt(1.0 + (pow(1.0 - ELLIPT, 2) - 1.0) * (1.0 - SINL * SINL));
	}

	MATRIX3 EquatorialToPlumbline(double LAT_GEOD, double LNG, double AZI)
	{
		// LAT_GEOD: Geodetic latitude, rad
		// LON: Longitude, rad
		// AZI: Launch azimuth, rad

		double CL, SL, CP, SP, CA, SA;

		CL = cos(LAT_GEOD);
		SL = sin(LAT_GEOD);
		CP = cos(LNG);
		SP = sin(LNG);
		CA = cos(AZI);
		SA = sin(AZI);

		return _M(CL * CP, CL * SP, SL, SA * SL * CP - CA * SP, SA * SL * SP + CA * CP, -SA * CL, -CA * SL * CP - SA * SP, -CA * SL * SP + SA * CP, CA * CL);
	}

	MATRIX3 EulerToDirectionCosineMatrix(double theta, double psi, double phi)
	{
		// INPUTS:
		// theta: pitch angle, rad
		// psi: yaw angle, rad
		// phi: roll angle, rad
		// OUTPUTS:
		// return value: matrix converting from body to inertial coordinates

		double CT, ST, CPSI, SPSI, CPHI, SPHI;

		CT = cos(theta);
		ST = sin(theta);
		CPSI = cos(psi);
		SPSI = sin(psi);
		CPHI = cos(phi);
		SPHI = sin(phi);

		return _M(CT * CPSI, ST * SPHI - CT * SPSI * CPHI, ST * CPHI + CT * SPSI * SPHI, SPSI, CPSI * CPHI, -CPSI * SPHI, -ST * CPSI, CT * SPHI + ST * SPSI * CPHI, CT * CPHI - ST * SPSI * SPHI);
	}

	void DirectionCosineMatrixToEuler(const MATRIX3& M, double& theta, double& psi, double& phi)
	{
		// INPUTS:
		// M: matrix converting from body to inertial coordinates
		// OUTPUTS:
		// theta: pitch angle, rad
		// psi: yaw angle, rad
		// phi: roll angle, rad

		psi = asin(M.m21);
		if (psi < 0.0)
		{
			psi += MathConstants::PI2;
		}

		theta = atan2(-M.m31, M.m11);
		if (theta < 0.0) theta += MathConstants::PI2;

		phi = atan2(-M.m23, M.m22);
		if (phi < 0.0) phi += MathConstants::PI2;
	}

	VECTOR3 EulersEquationInv(VECTOR3 I, VECTOR3 M, VECTOR3 W)
	{
		// INPUTS:
		// I: Moments of inertia
		// M: Moments
		// W: Body angular velocity
		// OUTPUTS:
		// W_dot: Body angular acceleration

		VECTOR3 W_dot;

		W_dot.x = (M.x - (I.z - I.y) * W.y * W.z) / I.x;
		W_dot.y = (M.y - (I.x - I.z) * W.z * W.x) / I.y;
		W_dot.z = (M.z - (I.y - I.x) * W.x * W.y) / I.z;

		return W_dot;
	}

	VECTOR3 BodyToEulerRates(VECTOR3 w_B, VECTOR3 Att)
	{
		// Att is P, Y, R

		VECTOR3 AttRate;
		double CPSI, TPSI, CPHI, SPHI;

		CPSI = cos(Att.y);
		TPSI = sin(Att.y) / CPSI;
		CPHI = cos(Att.z);
		SPHI = sin(Att.z);

		AttRate = _V(-w_B.z * SPHI / CPSI + w_B.y * CPHI / CPSI, w_B.y * SPHI + w_B.z * CPHI, w_B.x - w_B.y * TPSI * CPHI + w_B.z * TPSI * SPHI);

		return AttRate;
	}

	MATRIX3 InertialEquatorialToEarthFixed(double t, double w_E)
	{
		double CW, SW;

		CW = cos(w_E * t);
		SW = sin(w_E * t);

		return _M(CW, SW, 0.0, -SW, CW, 0.0, 0.0, 0.0, 1.0);
	}

	void PRA63(double Z, double& press, double& dens, double& spos)
	{
		// Patrick Air Force Base Reference Atmosphere 1963
		// INPUTS:
		// Z = Geometric altitude in meters
		// OUTPUTS:
		// press = Pressure in Pa
		// dens = Density in kg/m^3
		// spos = Speed of sound in m/s

		static const double PBASE = 6.23101759e-5;
		static const double ZI[5] = { 10832.1, 17853.3, 28000.0, 49000.0, 83004.0 };
		static const double PK[5][6] = { {  1.6871582e-2, -1.1425176e-4, -1.3612327e-9,  7.3624145e-14, -1.0800315e-17, 3.3046432e-22 },
											{ -7.9910777e-2, -8.1046438e-5, -5.5522383e-9,  3.1116969e-13, -1.6687827e-17, 3.8319351e-22 },
											{  9.8414277e-1, -2.6976917e-4,  8.5227541e-9, -3.9620263e-13, 1.0146471e-17, -1.0264318e-22 },
											{  1.14118495e1, -4.11497477e-4, 1.33664855e-8, -3.59518975e-13, 5.10097254e-18, -2.89055894e-23 },
											{  9.99324461e0, -2.58298177e-4, 3.76139346e-9, -4.20887236e-14, 1.60182148e-19, -1.92508927e-25} };
		static const double RHOK[3][6] = { { 1.3302117e-2, -8.8502064e-5, -4.2143056e-9, 5.9517557e-13, -3.9744789e-17, 7.8771273e-22 } ,
											{ 1.2667122e-1, -1.3373147e-4, 2.0667371e-9, 2.3396109e-13, -3.2562503e-17, 7.9035209e-22 } ,
											{ 9.2751266e-1, -1.4349679e-4, -2.8271736e-9, 4.7480092e-14, 1.8863246e-18, -4.2702411e-23 } };
		static const double TK[5][6] = { { 2.9667877e2, -6.7731001e-3, 8.4619805e-7, -1.7004049e-10, 1.1451454e-14, -2.4898788e-19 } ,
									{ 2.6892151e2, 4.3075352e-3, -8.9159672e-7, -2.8929791e-11, 5.0724856e-15, -1.1490372e-19 } ,
									{ 3.7064557e2, -3.2858965e-2, 2.0645636e-6, -4.3283944e-11, -5.7507242e-17, 8.2924583e-21 },
									{ 2.044798e1, 2.07698384e-2, -8.63038789e-7, 1.66392417e-11, -9.30076185e-17, -4.09005108e-22 },
									{ -4.98865953e2, 3.92137281e-2, -4.95180601e-7, -3.26219854e-12, 9.66650364e-17, -4.78844279e-22 } };
		static const double VTK[3][6] = { { 2.9937265e2, -7.717628e-3, 9.4867202e-7, -1.7136592e-10, 1.1074297e-14, -2.3294094e-19 } ,
									{ 2.6892151e2, 4.3075352e-3, -8.9159672e-7, -2.8929791e-11, 5.0724856e-15, -1.1490372e-19 } ,
									{ 3.7064557e2, -3.2858965e-2, 2.0645636e-6, -4.3283944e-11, -5.7507242e-17, 8.2924583e-21 } };

		static const double ZB[] = { 9.e4, 1.e5, 1.1e5, 1.2e5, 1.5e5, 1.6e5, 1.7e5, 1.9e5, 2.3e5, 3.0e5, 4.0e5, 5.0e5, 6.0e5, 7.0e5 };
		static const double TMB[] = { 180.65, 210.65, 260.65, 360.65, 960.65, 1110.65, 1210.65, 1350.65, 1550.65, 1830.65, 2160.65, 2420.65, 2590.65, 2700.65 };
		static const double LMB[] = { 3.e-3, 5.e-3, 10.e-3, 20.e-3, 15.e-3, 10.e-3, 7.e-3, 5.e-3, 4.e-3, 3.3e-3, 2.6e-3, 1.7e-3, 1.1e-3, 1.1e-3 };
		static const double PB[] = { 0.172244361e-4, 0.315971712e-5, 0.774389807e-6, 0.265977111e-6, 0.535849383e-7, 0.391284945e-7,  0.295911117e-7,
										0.178715656e-7, 0.739258171e-8, 0.200573116e-8, 0.430456606e-9, 0.117315480e-9, 0.370198961e-10, 0.128115330e-10 };

		if (Z < 0.0)
		{
			Z = 0.0;
		}
		if (Z >= ZB[13])
		{
			press = 0.0;
			dens = 0.0;
			spos = 1041.790289782520;
			return;
		}

		int N;

		if (Z < ZB[0])
		{
			// Below 90km

			double Z2, Z3, Z4, Z5, TEMPK, TEMPV;

			Z2 = Z * Z;
			Z3 = Z2 * Z;
			Z4 = Z2 * Z2;
			Z5 = Z2 * Z3;

			if (Z < ZI[0])
			{
				N = 0;
			}
			else if (Z < ZI[1])
			{
				N = 1;
			}
			else if (Z < ZI[2])
			{
				N = 2;
			}
			else if (Z < ZI[3])
			{
				N = 3;
			}
			else
			{
				N = 4;
			}

			// Pressure
			if (Z < ZI[2])
			{
				// 0 to 28,000 meters
				press = 10.0 * exp(PK[N][0] + PK[N][1] * Z + PK[N][2] * Z2 + PK[N][3] * Z3 + PK[N][4] * Z4 + PK[N][5] * Z5);
			}
			else if (Z < ZI[4])
			{
				// 28,000 to 83,004 meters
				press = 0.000980665 * exp(PK[N][0] + PK[N][1] * Z + PK[N][2] * Z2 + PK[N][3] * Z3 + PK[N][4] * Z4 + PK[N][5] * Z5);
			}
			else
			{
				// 83,004 to 90,000 meters
				press = PBASE * exp((-1.373301523e12 * (Z - 83004.0)) / (180.65 * (6344860.0 + Z) * (6344860.0 + 83004.0)));
			}

			// Kinetic temperature
			if (Z < ZI[4])
			{
				// 0 to 83,004 meters
				TEMPK = TK[N][0] + TK[N][1] * Z + TK[N][2] * Z2 + TK[N][3] * Z3 + TK[N][4] * Z4 + TK[N][5] * Z5;
			}
			else
			{
				// 83,004 to 90,000 meters
				TEMPK = 180.65;
			}

			// Virtual Temperature
			if (N <= 2)
			{
				TEMPV = VTK[N][0] + VTK[N][1] * Z + VTK[N][2] * Z2 + VTK[N][3] * Z3 + VTK[N][4] * Z4 + VTK[N][5] * Z5;
			}
			else
			{
				TEMPV = TEMPK;
			}

			// Density
			if (N <= 2)
			{
				// 0 to 28,000 meters
				dens = 1.16790729 * exp(RHOK[N][0] + RHOK[N][1] * Z + RHOK[N][2] * Z2 + RHOK[N][3] * Z3 + RHOK[N][4] * Z4 + RHOK[N][5] * Z5);
			}
			else
			{
				// 28,000 to 90,000 meters
				dens = 34.83676 * press / TEMPK;
			}

			// Speed of sound
			spos = 20.0468 * sqrt(TEMPV);
		}
		else
		{
			// Above 90km

			for (N = 0; (N < 12) && (ZB[N + 1] < Z); N++);

			double TEMPM, U, V;

			// Molecular temperature for (90000 to 700000)
			TEMPM = TMB[N] + LMB[N] * (Z - ZB[N]);
			// Kinetic temperature for (90000 to 700000)
			U = TMB[N] / (TMB[N] + LMB[N] * (Z - ZB[N]));
			V = 1.3733301523e12 / (LMB[N] * (Z + 6344860.0) * (6344860.0 + ZB[N]));
			press = PB[N] * pow(U, V);
			dens = 34.83676 * press / TEMPM;
			spos = 20.0468 * sqrt(TEMPM);
		}

		press *= 10000.0; // Convert to Pascal
	}

	void normalizeAngle(double& ang)
	{
		ang = std::fmod(ang, MathConstants::PI2);
		if (ang < 0.0)
		{
			ang += MathConstants::PI2;
		}
	}

	void normalizeAngle(VECTOR3& ang)
	{
		for (int i = 0; i < 3; i++)
		{
			normalizeAngle(ang.data[i]);
		}
	}

	double acos2(double _X)
	{
		return acos(std::fmin(1.0, std::fmax(-1.0, _X)));
	}
};