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

#include "SaturnLaunchSimulation.h"
#include "../util/Constants.h"
#include "SaturnV.h"
#include "IU/IU.h"

using namespace MathConstants;

SaturnLaunchSimulationInputs::SaturnLaunchSimulationInputs()
{
	t_Launch = 10380.0;
	t_Countdown = 600.0;

	lat_geod = 28.608419;
	lng = -80.604139;
	alt = 60.5;
	A_ZP = 90.0;

	IsSaturnV = true;
	SI_EmptyMass = 133602.0;
	SI_MainFuelMass = 2146040.0;
	SI_RetroFuelMass = 126.1;
	SI_RetroNum = 8;
	SII_EmptyMass = 39869.0;
	SII_MainFuelMass = 443500.0;
	SII_InterstageMass = 3982.0;
	SII_UllageFuelMass = 152.407;
	SII_UllageNum = 4;
	SII_RetroFuelMass = 121.65;
	SII_RetroNum = 4;
	S4B_EmptyMass = 13439.0;
	S4PL_Mass = 49794.0;
	S4B_MainFuelMass = 107428.0;
	S4B_UllageCaseEmptyMass = 62.0;
	S4B_UllageFuelMass = 26.67123;
	S4B_UllageNum = 2;
	S4B_APSPropellant = 143.0;
	LET_Mass = 4051.0;

	THRUST_FIRST_VAC = 7835000.0;
	ISP_FIRST_VAC = 2979.4;

	THRUST_SECOND_VAC = 1023000.0;
	ISP_SECOND_VAC = 418.0 * 9.80665;

	LVDCSoftwareName = "AS-512";

	gui_interface = NULL;
}

SaturnLaunchSimulation::SaturnLaunchSimulation(const OrbMech::GlobalConstants& constants) : cnst(constants)
{
	R_EF = _V(0, 0, 0);
	GMT_launch = 0.0;
	T = TLAST = 0.0;
	DT_GRR = 0.0;
	A_ZP = 0.0;
	MAT_AA = MAT_BB_init = _M(0, 0, 0, 0, 0, 0, 0, 0, 0);
	U_Z = W_B_pad = _V(0, 0, 0);
	lat_LS_geod = lng_LS = 0.0;

	sat = NULL;
}

SaturnLaunchSimulation::~SaturnLaunchSimulation()
{
	if (sat) delete sat;
}

int SaturnLaunchSimulation::Init(const SaturnLaunchSimulationInputs& in)
{
	if (in.IsSaturnV)
	{
		sat = new SaturnV;
	}
	else
	{
		// TBD
	}
	if (sat->Init(&in))
	{
		return 1;
	}

	GMT_launch = in.t_Launch;
	T = in.t_Launch - in.t_Countdown;
	TLAST = T;

	DT_GRR = 17.0;

	// Convert inputs
	lat_LS_geod = in.lat_geod * RAD_PER_DEG;
	lng_LS = in.lng * RAD_PER_DEG;
	A_ZP = in.A_ZP * RAD_PER_DEG;

	// Calculate R_EF, W_B
	R_EF = OrbMech::GEODETIC_TO_EF(lat_LS_geod, lng_LS, in.alt, cnst.ELLIPT, cnst.R_E);

	InitTrajectory(90.0 * RAD_PER_DEG);

	// Initialize derived variables
	PrelaunchStep(0.0);

	return 0;
}

void SaturnLaunchSimulation::InitTrajectory(double A_Z)
{
	// Calculate: U_Z, AA, BB_init
	MAT_AA = OrbMech::EquatorialToPlumbline(lat_LS_geod, lng_LS, A_Z);
	U_Z = mul(MAT_AA, _V(0.0, 0.0, 1.0));
	MAT_BB_init = OrbMech::EulerToDirectionCosineMatrix(0.0, 0.0, A_ZP + A_Z + PI);

	// This doesn't actually depend on launch azimuth, but it's convenient to calculate here
	VECTOR3 W_E_U = _V(0.0, 0.0, cnst.w_E);
	VECTOR3 W_E_S = mul(MAT_AA, W_E_U);
	W_B_pad = tmul(MAT_BB_init, W_E_S);
}

void SaturnLaunchSimulation::Step(double DT)
{
	// Call subsystem pre-step functions
	sat->UpdateState(CurrentState.R, CurrentState.V, CurrentState.Att, CurrentState.AttRate, Measured_Acceleration);
	sat->PreStep(T, DT);

	if (GetCountdownTime() < 0.0)
	{
		PrelaunchStep(DT);
	}
	else
	{
		InFlightStep(DT);
	}
	// Normalize angles
	OrbMech::normalizeAngle(CurrentState.Att);
	// Increment time
	T = T + DT;

	// Call subsystem post-step functions
	sat->UpdateState(CurrentState.R, CurrentState.V, CurrentState.Att, CurrentState.AttRate, Measured_Acceleration);
	sat->PostStep(T, DT);
	sat->ResetAdditionalForces();

	CalculateDisplayParameters();
}

void SaturnLaunchSimulation::PrelaunchStep(double DT)
{
	// Calculate time relative to GRR
	double t = (T + DT) - (GMT_launch - DT_GRR);

	// Calculate Earth rotation matrix
	MATRIX3 D_apo = OrbMech::InertialEquatorialToEarthFixed(t, cnst.w_E);

	// Calculate position vector
	CurrentState.R = mul(MAT_AA, tmul(D_apo, R_EF));
	// Calculate velocity vector
	CurrentState.V = crossp(U_Z, CurrentState.R) * cnst.w_E;
	// Calculate body to plumbline(?) matrix
	MATRIX3 M2 = mul(MAT_AA, mul(tmat(D_apo), mul(tmat(MAT_AA), MAT_BB_init)));
	// Extract Euler angles
	OrbMech::DirectionCosineMatrixToEuler(M2, CurrentState.Att.x, CurrentState.Att.y, CurrentState.Att.z);
	// Body rate is fixed
	CurrentState.AttRate = W_B_pad;

	// Call Accel function once to update any derived variables
	Accel(T + DT, CurrentState);
}

void SaturnLaunchSimulation::InFlightStep(double DT)
{
	// Runge-Kutta-Gill
	StateVector k1, k2, k3, k4;

	// First evaluation
	k1 = Accel(T + DT * RKG_A[0], CurrentState) * DT;
	// Second evaluation
	k2 = Accel(T + DT * RKG_A[1], CurrentState + k1 * RKG_B[0]) * DT;
	// Third evaluation
	k3 = Accel(T + DT * RKG_A[2], CurrentState + k1 * RKG_B[1] + k2 * RKG_B[2]) * DT;
	// Fourth evaluation
	k4 = Accel(T + DT * RKG_A[3], CurrentState + k1 * RKG_B[3] + k2 * RKG_B[4] + k3 * RKG_B[5]) * DT;

	// Update state
	CurrentState = CurrentState + (k1 * RKG_C[0] + k2 * RKG_C[1] + k3 * RKG_C[2] + k4 * RKG_C[3]);

	// Call Accel function once to update any derived variables
	Accel(T + DT, CurrentState);
}

SaturnLaunchSimulation::StateVector SaturnLaunchSimulation::Accel(double t, const SaturnLaunchSimulation::StateVector& y)
{
	StateVector DERIV;
	MATRIX3 M_B_P;
	VECTOR3 R5, a_total, a_grav, a_measured, EulerRates, W_B_dot, CG, PMI, F_thrust_sum_B, M_thrust_sum_B, V_rel, V_a, F_thrust_sum_I, M_sum, F_aero_B, F_aero_I, r_m, M_A_m;
	double Mass, alt, press, dens, spos, v_rel, Q, alpha, beta, Mach, CN, CA, X_CP, Area, F_A, F_N;

	// Rotation matrices
	M_B_P = OrbMech::EulerToDirectionCosineMatrix(y.Att.x, y.Att.y, y.Att.z);

	// Time dependent update
	if (t != TLAST)
	{
		// Mass update, propellant etc.
		sat->UpdatePropellants(t - TLAST);

		TLAST = t;
	}
	sat->UpdateMass();
	Mass = sat->GetMass();

	// Calculate CG location
	CG = sat->CalculateCG();

	// Calculate moments of inertia
	PMI = sat->CalculatePMI();

	// Gravity acceleration
	a_grav = GravityModel(y.R);

	// Calculate position vector in equatorial coordinates
	R5 = tmul(MAT_AA, y.R);

	// Calculate altitude
	alt = length(R5) - OrbMech::RadiusEllipsoid(R5, cnst.R_E, cnst.ELLIPT);

	// Calculate atmospheric parameters
	OrbMech::PRA63(alt, press, dens, spos);

	// Calculate velocity vector relative to wind
	V_rel = y.V - crossp(U_Z * cnst.w_E, y.R);
	v_rel = length(V_rel);

	// Dynamic pressure
	Q = dens * v_rel * v_rel * 0.5;

	// Mach number
	Mach = v_rel / spos;

	// Convert V_rel to body coordinates
	V_a = tmul(M_B_P, V_rel);

	// Angle of attack
	if (v_rel < 1e-10)
	{
		alpha = 0.0;
	}
	else
	{
		alpha = acos(V_a.x / v_rel);
	}
	beta = atan2(V_a.z, V_a.y);

	// Aerodynamic coefficients, center of pressure
	sat->GetAerodynamicCoefficients(alpha, Mach, CN, CA, X_CP, Area);

	// Calculate aerodynamic forces
	F_N = CN * Q * Area;
	F_A = CA * Q * Area;

	// Calculate aerodynamic force vector
	F_aero_B = _V(-F_A, -F_N * cos(beta), -F_N * sin(beta));

	// Convert to inertial coordinates
	F_aero_I = mul(M_B_P, F_aero_B);

	// Calculate aerodynamic moment arm
	r_m = _V(X_CP, 0.0, 0.0) - CG;

	// Calculate aerodynamic moment vector
	M_A_m = crossp(r_m, F_aero_B);

	// Thrust
	sat->UpdateThrustForces(CG, press, F_thrust_sum_B, M_thrust_sum_B);

	// Convert thrust to inertial coordinates
	F_thrust_sum_I = mul(M_B_P, F_thrust_sum_B);

	if (GetCountdownTime() < 0.0)
	{
		// Before launch, hold-down and centrifugal acceleration
		VECTOR3 a_holddown, a_centr, WWW;

		a_holddown = -a_grav;

		// Centrifugal acceleration
		WWW = U_Z * cnst.w_E;
		a_centr = crossp(WWW, crossp(WWW, y.R));

		// Total acceleration
		a_measured = a_holddown + a_centr;
	}
	else
	{
		// After launch, thrust and aerodynamic forces
		a_measured = (F_thrust_sum_I + F_aero_I) / Mass;
	}

	// Total acceleration
	a_total = a_grav + a_measured;

	// Convert body rates to Euler rates
	EulerRates = OrbMech::BodyToEulerRates(y.AttRate, y.Att);

	// Angular acceleration
	M_sum = M_thrust_sum_B + M_A_m;
	W_B_dot = OrbMech::EulersEquationInv(PMI, M_sum, y.AttRate);

	// Derived variables
	Measured_Acceleration = a_measured;

	// Store some parameters for display
	out.traj.F = length(F_thrust_sum_I);
	out.traj.Q = Q;
	out.traj.alpha = alpha * MathConstants::DEG_PER_RAD;
	out.traj.F_aero = length(F_aero_B);

	// Collect all derivatives
	DERIV.R = y.V;
	DERIV.V = a_total;
	DERIV.Att = EulerRates;
	DERIV.AttRate = W_B_dot;

	return DERIV;
}

VECTOR3 SaturnLaunchSimulation::GravityModel(const VECTOR3& R) const
{
	VECTOR3 U_R, a_dE, a_dP;
	double r, cos_lat, P2, P3, P4, P5;

	r = length(R);
	U_R = R / r;
	cos_lat = dotp(U_R, U_Z);

	P2 = 3.0 * cos_lat;
	P3 = 0.5 * (15.0 * cos_lat * cos_lat - 3.0);
	P4 = 1.0 / 3.0 * (7.0 * cos_lat * P3 - 4.0 * P2);
	P5 = 0.25 * (9.0 * cos_lat * P4 - 5.0 * P3);

	a_dE = (U_R * P3 - U_Z * P2) * cnst.J2 * pow(cnst.R_E / r, 2);
	a_dE += (U_R * P4 - U_Z * P3) * cnst.J3 * pow(cnst.R_E / r, 3);
	a_dE += (U_R * P5 - U_Z * P4) * cnst.J4 * pow(cnst.R_E / r, 4);
	a_dE = a_dE * cnst.MU / (r * r);

	a_dP = -R * cnst.MU / pow(r, 3);

	return a_dE + a_dP;
}

void SaturnLaunchSimulation::CalculateDisplayParameters()
{
	MATRIX3 M_5_EF;
	VECTOR3 U_R, R5, R_EF;
	double LAT_GEOD, LON, ALT;

	U_R = unit(CurrentState.R);

	out.traj.T = GetCountdownTime();
	out.traj.V = length(CurrentState.V);
	out.traj.Hdot = dotp(CurrentState.V, U_R);
	out.traj.m = sat->GetMass();

	VECTOR3 LVIMU_Att = sat->GetIU()->GetLVDA()->GetAttitude();

	out.traj.Roll = LVIMU_Att.x * DEG_PER_RAD;
	out.traj.Pitch = LVIMU_Att.y * DEG_PER_RAD;
	out.traj.Yaw = LVIMU_Att.z * DEG_PER_RAD;

	out.traj.Rates[0] = CurrentState.AttRate.x * DEG_PER_RAD;
	out.traj.Rates[1] = CurrentState.AttRate.y * DEG_PER_RAD;
	out.traj.Rates[2] = CurrentState.AttRate.z * DEG_PER_RAD;

	R5 = tmul(MAT_AA, CurrentState.R);

	M_5_EF = OrbMech::InertialEquatorialToEarthFixed(T - (GMT_launch - DT_GRR), cnst.w_E);
	R_EF = mul(M_5_EF, R5);

	OrbMech::EF_TO_GEODETIC(R_EF, LAT_GEOD, LON, ALT, cnst.ELLIPT, cnst.R_E);

	out.traj.Latitude = LAT_GEOD * DEG_PER_RAD;
	out.traj.Longitude = LON * DEG_PER_RAD;
	out.traj.Alt = ALT / 1000.0;

	out.rca.TestProgram = sat->GetML()->GetRCA()->GetProgram();
	out.rca.State = sat->GetML()->GetRCA()->GetState();
	out.rca.SubState = sat->GetML()->GetRCA()->GetSubState();
	out.rca.ModeRegister = sat->GetML()->GetRCA()->GetModeRegister();
	out.rca.StatusCode = sat->GetML()->GetRCA()->GetStatusCode();
	sat->GetML()->GetRCA()->GetLVDAData(out.rca.LastPIO, out.rca.LastPIOValue);
	out.rca.LVDCErrorMessage = sat->GetML()->GetRCA()->GetLVDCErrorMessage();
}

double SaturnLaunchSimulation::GetGMT()
{
	return T;
}

double SaturnLaunchSimulation::GetCountdownTime()
{
	return (T - GMT_launch);
}