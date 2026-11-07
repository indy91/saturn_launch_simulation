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
#include "OrbMech.h"
#include <string>

class Saturn;
class LaunchSimGUIInterface;

struct SaturnLaunchSimulationInputs
{
	SaturnLaunchSimulationInputs();

	// TIMES
	double t_Launch;	// Launch GMT, seconds
	double t_Countdown;	// Initial time until launch (positive value), seconds

	// INITIAL STATE
	double lat_geod;		// Geodetic launch site latitude, degrees
	double lng;				// Geodetic launch site longitude, degrees
	double alt;
	double A_ZP;			// Launchpad orientation, degrees

	// MASSES
	bool IsSaturnV;
	double SI_EmptyMass;
	double SI_MainFuelMass;
	double SI_RetroFuelMass;
	int SI_RetroNum;
	double SII_EmptyMass;
	double SII_MainFuelMass;
	double SII_InterstageMass;
	double SII_UllageFuelMass;
	double SII_RetroFuelMass;
	int SII_UllageNum;
	int SII_RetroNum;
	double S4B_EmptyMass;			// S-IVB empty mass (including IU and SLA), kilograms
	double S4B_MainFuelMass;
	double S4B_UllageFuelMass;
	double S4B_UllageCaseEmptyMass;
	int S4B_UllageNum;
	double S4B_APSPropellant;
	double LET_Mass;
	double S4PL_Mass;				// Payload mass (CSM + LM), kilograms

	// PROPULSION
	double THRUST_FIRST_VAC;
	double ISP_FIRST_VAC;
	double THRUST_SECOND_VAC;
	double ISP_SECOND_VAC;

	// FILES
	std::string LVDCSoftwareName;

	// POINTERS
	LaunchSimGUIInterface* gui_interface;
};

struct TrajectoryDataOutput
{
	double T;				// Time from first motion, seconds
	double V;				// Inertial velocity, m/s
	double Alt;				// Altitude above equator, kilometers
	double Hdot;			// Altitude rate, m/s
	double F;				// Sum of thrust forces, N
	double m;				// Total mass, kg
	double Roll, Pitch, Yaw;// Platform attitude, degrees
	double Rates[3];		// Roll, pitch, yaw body rates, deg/sec
	double Q;				// Dynamic pressure, TBD
	double F_aero;			// Aerodynamic force, Newton
	double alpha;			// Angle of attack, degrees
	double beta_y[5];		// Yaw actuator angles, degrees
	double beta_p[5];		// Pitch actuator angles, degrees
	double Latitude;
	double Longitude;
};

struct GroundComputerOutputs
{
	std::string TestProgram;
	int State;
	int SubState;

	int ModeRegister;
	int StatusCode;
	int LastPIO;
	int LastPIOValue;
	int LVDCErrorMessage;
};

struct SaturnLaunchSimulationOutputs
{
	TrajectoryDataOutput traj;
	GroundComputerOutputs rca;
};

class SaturnLaunchSimulation
{
public:
	SaturnLaunchSimulation(const OrbMech::GlobalConstants& constants);
	~SaturnLaunchSimulation();

	int Init(const SaturnLaunchSimulationInputs& in);

	// Initialize trajectory
	void InitTrajectory(double A_Z);

	// Perform one simulation timestep
	void Step(double DT);

	// FOR SUBSYSTEM
	double GetGMT();
	double GetCountdownTime();

	SaturnLaunchSimulationOutputs out;
private:

	struct StateVector
	{
		union
		{
			double data[12];
			struct
			{
				VECTOR3 R; // Position in plumbline coordinates
				VECTOR3 V; // Velocity in plumbline coordinates
				VECTOR3 Att; // Attitude in plumbline coordinates (P, Y, R)
				VECTOR3 AttRate; // Attitude rate in body coordinates (X, Y, Z)
			};
		};

		StateVector operator+(const StateVector& a)
		{
			StateVector c;
			for (int i = 0; i < 12; i++)
			{
				c.data[i] = data[i] + a.data[i];
			}
			return c;
		}

		StateVector operator* (const double f)
		{
			StateVector c;
			for (int i = 0; i < 12; i++)
			{
				c.data[i] = data[i] * f;
			}
			return c;
		}
	};

	// Perform an in-flight trajectory step
	void InFlightStep(double DT);
	// Perform a prelaunch step
	void PrelaunchStep(double DT);

	// Derivatives Function
	StateVector Accel(double t, const StateVector& y);

	// FORCES
	// Gravity
	VECTOR3 GravityModel(const VECTOR3& R) const;

	// Display parameters
	void CalculateDisplayParameters();

	// STATE OF SIMULATION
	StateVector CurrentState;
	double T;
	double TLAST;
	VECTOR3 Measured_Acceleration;

	// ADDITIONAL VARIABLES
	// GMT of launch
	double GMT_launch;
	// Earth-fixed position of launch, meters
	VECTOR3 R_EF;
	// Inertial equatorial to plumbline coordinates
	MATRIX3 MAT_AA;
	// Geodetic latitude of launch site, rad
	double lat_LS_geod;
	// Longitude of launch site, rad
	double lng_LS;
	// Orientation on launchpad, rad
	double A_ZP;
	// Polar unit vector
	VECTOR3 U_Z;
	// Body to Earth-fixed plumbline coordinates
	MATRIX3 MAT_BB_init;
	// Body attitude rates on launchpad, rad/s
	VECTOR3 W_B_pad;
	// Time from GRR to launch, seconds
	double DT_GRR;

	// VEHICLE
	Saturn* sat;

	// CONSTANTS
	const OrbMech::GlobalConstants cnst;
	// Runge-Kutta-Gill integration constants
	const double RKG_A[4] = { 0.0, 1.0 / 2.0, 1.0 / 2.0, 1.0 };
	const double RKG_B[6] = { 1.0 / 2.0, (-1.0 + sqrt(2.0)) / 2.0, (2.0 - sqrt(2.0)) / 2.0, 0.0, -sqrt(2.0) / 2.0, (2.0 + sqrt(2.0)) / 2.0 };
	const double RKG_C[4] = { 1.0 / 6.0, (2.0 - sqrt(2.0)) / 6.0, (2.0 + sqrt(2.0)) / 6.0, 1.0 / 6.0 };
};