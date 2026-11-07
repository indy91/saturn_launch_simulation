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

#include "SaturnV.h"
#include "SaturnLaunchSimulation.h"
#include "S-IC/s1csystems.h"
#include "S-II/siisystems.h"
#include "S-IVB/sivbsystems.h"
#include "IU/IU.h"
#include "../util/Constants.h"

SaturnV::SaturnV()
{
	SII_EmptyMass = 0.0;

	SII_UllageNum = 0;
	SII_RetroNum = 0;

	SIC = NULL;
	SII_IS = NULL;
	SII = NULL;
	SIVB = NULL;
	IU = NULL;
}

SaturnV::~SaturnV()
{
	if (SIC)
	{
		delete SIC;
		SIC = NULL;
	}
	if (SII_IS)
	{
		delete SII_IS;
		SII_IS = NULL;
	}
	if (SII)
	{
		delete SII;
		SII = NULL;
	}
	if (SIVB)
	{
		delete SIVB;
		SIVB = NULL;
	}
	if (IU)
	{
		delete IU;
		IU = NULL;
	}
}

int SaturnV::Init(const SaturnLaunchSimulationInputs* in)
{
	Saturn::Init(in);

	SII_EmptyMass = in->SII_EmptyMass;
	SII_UllageNum = in->SII_UllageNum;
	SII_RetroNum = in->SII_RetroNum;

	SIIMainPropellant = CreatePropellantResource(in->SII_MainFuelMass);
	for (int i = 0; i < in->SII_RetroNum; i++)
	{
		SIIRetroRocketPropellant[i] = CreatePropellantResource(in->SII_RetroFuelMass);
	}
	for (int i = 0; i < in->SII_UllageNum; i++)
	{
		SIIUllageRocketPropellant[i] = CreatePropellantResource(in->SII_UllageFuelMass);
	}

	double cl = 1664.0 * 0.0254;
	double d = 74.246 * 0.0254;
	double a = 3.24;

	SIIJ2Engines[0] = CreateThruster(_V(cl, -d, d), _V(1.0, 0.0, 0.0), in->THRUST_SECOND_VAC, &SIIMainPropellant, in->ISP_SECOND_VAC, a);
	SIIJ2Engines[1] = CreateThruster(_V(cl, -d, -d), _V(1.0, 0.0, 0.0), in->THRUST_SECOND_VAC, &SIIMainPropellant, in->ISP_SECOND_VAC, a);
	SIIJ2Engines[2] = CreateThruster(_V(cl, d, -d), _V(1.0, 0.0, 0.0), in->THRUST_SECOND_VAC, &SIIMainPropellant, in->ISP_SECOND_VAC, a);
	SIIJ2Engines[3] = CreateThruster(_V(cl, d, d), _V(1.0, 0.0, 0.0), in->THRUST_SECOND_VAC, &SIIMainPropellant, in->ISP_SECOND_VAC, a);
	SIIJ2Engines[4] = CreateThruster(_V(cl, 0.0, 0.0), _V(1.0, 0.0, 0.0), in->THRUST_SECOND_VAC, &SIIMainPropellant, in->ISP_SECOND_VAC, a);

	for (int i = 0; i < SII_UllageNum; i++)
	{
		SIIUllageRockets[i] = CreateThruster(_V(cl, 0.0, 0.0), _V(1.0, 0.0, 0.0), 23000.0 * 4.4482216152605, &SIIUllageRocketPropellant[i], 2517.3, 0.0);
	}
	for (int i = 0; i < SII_RetroNum; i++)
	{
		SIIRetroRockets[i] = CreateThruster(_V(cl + 10.0, 0.0, 0.0), _V(-1.0, 0.0, 0.0), 34810.0 * 4.4482216152605, &SIIRetroRocketPropellant[i], 2517.3, 0.0);
	}

	// S-IVB
	SIVBJ2Engine = CreateThruster(_V(2645.853 * 0.0254, 0.0, 0.0), _V(1.0, 0.0, 0.0), in->THRUST_SECOND_VAC, &SIVBMainPropellant, in->ISP_SECOND_VAC, a);
	for (int i = 0; i < S4B_UllageNum; i++)
	{
		SIVBUllageRockets[i] = CreateThruster(_V(2645.853 * 0.0254, 0.0, 0.0), _V(1.0, 0.0, 0.0), 3390.0 * 4.4482216152605, &SIVBUllageRocketPropellant[i], 2517.3, 0.0);
	}

	const VECTOR3 APS_DIRECTIONS[6] = {
	{0.0, 0.0, -1.0},		// I p
	{0.0, 0.0, 1.0 },		// III p
	{0.0, 1.0, 0.0 },		// III II
	{0.0, -1.0, 0.0 },		// III IV
	{0.0, 1.0, 0.0 },		// I II
	{0.0, -1.0, 0.0 } };	// I IV
	const VECTOR3 APS_LOCATIONS[6] = {
		{2829.0 * 0.0254, 0.0, 130.0 * 0.0254},
		{2829.0 * 0.0254, 0.0, -130.0 * 0.0254},
		{2829.0 * 0.0254, 0.0, -130.0 * 0.0254},
		{2829.0 * 0.0254, 0.0, -130.0 * 0.0254},
		{2829.0 * 0.0254, 0.0, 130.0 * 0.0254},
		{2829.0 * 0.0254, 0.0, 130.0 * 0.0254}
	};

	SIVBAPSEngines[0] = CreateThruster(APS_LOCATIONS[0], APS_DIRECTIONS[0], 150.0 * 4.4482216152605, &SIVBAPSPropellant[0], 2517.3, 0.0);
	SIVBAPSEngines[1] = CreateThruster(APS_LOCATIONS[1], APS_DIRECTIONS[1], 150.0 * 4.4482216152605, &SIVBAPSPropellant[1], 2517.3, 0.0);
	SIVBAPSEngines[2] = CreateThruster(APS_LOCATIONS[2], APS_DIRECTIONS[2], 150.0 * 4.4482216152605, &SIVBAPSPropellant[1], 2517.3, 0.0);
	SIVBAPSEngines[3] = CreateThruster(APS_LOCATIONS[3], APS_DIRECTIONS[3], 150.0 * 4.4482216152605, &SIVBAPSPropellant[1], 2517.3, 0.0);
	SIVBAPSEngines[4] = CreateThruster(APS_LOCATIONS[4], APS_DIRECTIONS[4], 150.0 * 4.4482216152605, &SIVBAPSPropellant[0], 2517.3, 0.0);
	SIVBAPSEngines[5] = CreateThruster(APS_LOCATIONS[5], APS_DIRECTIONS[5], 150.0 * 4.4482216152605, &SIVBAPSPropellant[0], 2517.3, 0.0);

	SIVBAPSUllageEngines[0] = CreateThruster(APS_LOCATIONS[0], _V(1.0, 0.0, 0.0), 70.0 * 4.4482216152605, &SIVBAPSPropellant[0], 2517.3, 0.0);
	SIVBAPSUllageEngines[1] = CreateThruster(APS_LOCATIONS[1], _V(1.0, 0.0, 0.0), 70.0 * 4.4482216152605, &SIVBAPSPropellant[1], 2517.3, 0.0);

	SIC = new SICSystems(this, SIEngines, &SIMainPropellant);
	SII_IS = new SIIInterstageSystems();
	SII = new SIISystems(this, SIIJ2Engines, &SIIMainPropellant, SIIUllageRockets, SII_UllageNum);
	SII->RecalculateEngineParameters(in->THRUST_SECOND_VAC);
	SIVB = new SIVB500Systems(this, &SIVBJ2Engine, &SIVBMainPropellant, SIVBAPSEngines, SIVBAPSUllageEngines, SIVBUllageRockets, S4B_UllageNum);
	((SIVBSystems*)SIVB)->RecalculateEngineParameters(in->THRUST_SECOND_VAC);
	IU = new IUSV(this);

	// Wire together connectors
	ml.ConnectSIUmbilical(SIC->GetSICToSIESEConnector());
	ml.ConnectIUUmbilical(IU->GetIUToIUESECommandConnector());
	SIC->GetSICToSIIConnector()->ConnectTo(SII_IS->GetSIIInterstageToSICConnector());
	SII_IS->GetSIIInterstageToSIIConnector()->ConnectTo(SII->GetSIIToSIIInterstageConnector());
	SII->GetSIIToSIVBConnector()->ConnectTo(SIVB->GetSIVBSIXConnector());
	SIVB->GetSIVBIUConnector()->ConnectTo(IU->GetSIVBCommandConnector());

	// Init functions
	return IU->SetMissionInfo(true, true, in->LVDCSoftwareName);
}

void SaturnV::PreStep(double t, double dt)
{
	Saturn::PreStep(t, dt);

	if (stage <= LAUNCH_STAGE_ONE) SIC->Timestep(t, dt);
	if (stage <= LAUNCH_STAGE_TWO) SII->Timestep(dt);
	SIVB->Timestep(dt);
	IU->PreStep(t, dt);
}

void SaturnV::PostStep(double t, double dt)
{
	Saturn::PostStep(t, dt);

	IU->PostStep(t, dt);

	// Staging Logic
	if (stage == PRELAUNCH_STAGE && (t - GMTLO) > 0.0)
	{
		ml.DisconnectUmbilicals();
		stage = LAUNCH_STAGE_ONE;

		SendEventMessage("Liftoff");
	}

	if (stage == LAUNCH_STAGE_ONE && SIC->IsPyroBlown())
	{
		SII_IS->GetSIIInterstageToSICConnector()->Disconnect();
		stage = LAUNCH_STAGE_TWO;

		SendEventMessage("S-IC/S-II Separation");
	}

	// Interstage Jettison
	if (SII->GetSIIToSIIInterstageConnector()->connectedTo != NULL && SII->SII_Interstage_Pyros.IsBlown())
	{
		bInterstageAttached = false;
		SII->GetSIIToSIIInterstageConnector()->Disconnect();

		SendEventMessage("Interstage Separation");
	}

	// LET Jettison
	if (bLETAttached && (t - GMTLO) > 199.0)
	{
		bLETAttached = false;

		SendEventMessage("LET Jettison");
	}

	if (stage == LAUNCH_STAGE_TWO && SII->IsPyroBlown())
	{
		SIVB->GetSIVBSIXConnector()->Disconnect();
		stage = LAUNCH_STAGE_SIVB;

		SendEventMessage("S-II/S-IVB Separation");
	}

	if (bSIVBUllageCasingsAttached && SIVB->Is_Ullage_Jettison_PyrosBlown())
	{
		bSIVBUllageCasingsAttached = false;

		SendEventMessage("S-IVB Ullage Jettison");
	}
}

void SaturnV::UpdatePropellants(double simdt)
{
	if (stage <= LAUNCH_STAGE_ONE)
	{
		for (int i = 0; i < 5; i++)
		{
			UpdateThruster(&SIEngines[i], simdt);
		}
		for (int i = 0; i < SI_RetroNum; i++)
		{
			UpdateThruster(&SIRetroRockets[i], simdt);
		}
	}
	if (stage <= LAUNCH_STAGE_TWO)
	{
		for (int i = 0; i < 5; i++)
		{
			UpdateThruster(&SIIJ2Engines[i], simdt);
		}
		for (int i = 0; i < SII_RetroNum; i++)
		{
			UpdateThruster(&SIIRetroRockets[i], simdt);
		}
		for (int i = 0; i < SII_UllageNum; i++)
		{
			UpdateThruster(&SIIUllageRockets[i], simdt);
		}
	}

	Saturn::UpdatePropellants(simdt);
}

void SaturnV::UpdateThrustForces(const VECTOR3& CG, double press, VECTOR3& F_sum, VECTOR3& M_sum) const
{
	VECTOR3 F, M;

	F_sum = M_sum = _V(0, 0, 0);

	if (stage <= LAUNCH_STAGE_ONE)
	{
		for (int i = 0; i < 5; i++)
		{
			ThrusterForceMoment(&SIEngines[i], CG, press, F, M);
			F_sum += F;
			M_sum += M;
		}
		for (int i = 0; i < SI_RetroNum; i++)
		{
			ThrusterForceMoment(&SIRetroRockets[i], CG, press, F, M);
			F_sum += F;
			M_sum += M;
		}
	}
	if (stage <= LAUNCH_STAGE_TWO)
	{
		for (int i = 0; i < 5; i++)
		{
			ThrusterForceMoment(&SIIJ2Engines[i], CG, press, F, M);
			F_sum += F;
			M_sum += M;
		}
		for (int i = 0; i < SII_UllageNum; i++)
		{
			ThrusterForceMoment(&SIIUllageRockets[i], CG, press, F, M);
			F_sum += F;
			M_sum += M;
		}
		for (int i = 0; i < SII_RetroNum; i++)
		{
			ThrusterForceMoment(&SIIRetroRockets[i], CG, press, F, M);
			F_sum += F;
			M_sum += M;
		}
	}

	ThrusterForceMoment(&SIVBJ2Engine, CG, press, F, M);
	F_sum += F;
	M_sum += M;

	for (int i = 0; i < S4B_UllageNum; i++)
	{
		ThrusterForceMoment(&SIVBUllageRockets[i], CG, press, F, M);
		F_sum += F;
		M_sum += M;
	}
	for (int i = 0; i < 6; i++)
	{
		ThrusterForceMoment(&SIVBAPSEngines[i], CG, press, F, M);
		F_sum += F;
		M_sum += M;
	}
	for (int i = 0; i < 2; i++)
	{
		ThrusterForceMoment(&SIVBAPSUllageEngines[i], CG, press, F, M);
		F_sum += F;
		M_sum += M;
	}

	// Additional forces
	F_sum += Flin_add;
	M_sum += Amom_add;
}

void SaturnV::UpdateMass()
{
	double emptymass = 0.0, fuelmass = 0.0;

	// Empty mass
	if (stage <= LAUNCH_STAGE_ONE)
	{
		emptymass += SI_EmptyMass;
	}
	if (stage <= LAUNCH_STAGE_TWO)
	{
		emptymass += SII_EmptyMass;
		if (bInterstageAttached)
		{
			emptymass += Interstage_Mass;
		}
	}
	emptymass += (S4B_EmptyMass + S4PL_Mass);
	if (bSIVBUllageCasingsAttached)
	{
		emptymass += (S4B_UllageCaseEmptyMass * (double)S4B_UllageNum);
	}
	if (bLETAttached)
	{
		emptymass += LET_Mass;
	}

	// Propellant
	if (stage <= LAUNCH_STAGE_ONE)
	{
		fuelmass += SIMainPropellant.mass;
		for (int i = 0; i < SI_RetroNum; i++)
		{
			fuelmass += SIRetroRocketPropellant[i].mass;
		}
	}
	if (stage <= LAUNCH_STAGE_TWO)
	{
		fuelmass += SIIMainPropellant.mass;
		for (int i = 0; i < SII_UllageNum; i++)
		{
			fuelmass += SIIUllageRocketPropellant[i].mass;
		}
		for (int i = 0; i < SII_RetroNum; i++)
		{
			fuelmass += SIIRetroRocketPropellant[i].mass;
		}
	}
	fuelmass += SIVBMainPropellant.mass;
	for (int i = 0; i < S4B_UllageNum; i++)
	{
		fuelmass += SIVBUllageRocketPropellant[i].mass;
	}
	fuelmass += SIVBAPSPropellant[0].mass;
	fuelmass += SIVBAPSPropellant[1].mass;

	SetMass(emptymass + fuelmass);
}

VECTOR3 SaturnV::CalculateCG() const
{
	// TBD: Stage dependent table as a function of mass

	VECTOR3 CG;

	if (stage <= LAUNCH_STAGE_ONE)
	{
		// S-IC
		CG = _V(30.0, 0, 0); //TBD
	}
	else if (stage == LAUNCH_STAGE_TWO)
	{
		// S-II
		CG = _V(60.0, 0, 0); //TBD
	}
	else
	{
		// S-IVB
		CG = _V(80.0, 0, 0); //TBD
	}

	return CG;
}

VECTOR3 SaturnV::CalculatePMI() const
{
	VECTOR3 pmi;

	if (stage <= LAUNCH_STAGE_ONE)
	{
		// S-IC
		pmi = _V(5.875, 440.0, 440.0) * GetMass(); // TBD
	}
	else if (stage == LAUNCH_STAGE_TWO)
	{
		// S-II
		pmi = _V(5.875, 374.0, 374.0) * GetMass(); // TBD
	}
	else
	{
		// S-IVB
		pmi = _V(5.0, 53.5, 53.5) * GetMass();  // TBD
	}

	return pmi;
}

void SaturnV::GetAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const
{
	alpha *= MathConstants::DEG_PER_RAD;

	if (stage <= LAUNCH_STAGE_ONE)
	{
		GetSICAerodynamicCoefficients(alpha, Mach, CN, CA, CP, Area);
	}
	else if (stage == LAUNCH_STAGE_TWO)
	{
		GetSIIAerodynamicCoefficients(alpha, Mach, CN, CA, CP, Area);
	}
	else
	{
		GetSIVBAerodynamicCoefficients(alpha, Mach, CN, CA, CP, Area);
	}
}

double BilinearInterpolation(double q11, double q12, double q21, double q22, double x1, double x2, double y1, double y2, double x, double y)
{
	double x2x1, y2y1, x2x, y2y, yy1, xx1;
	x2x1 = x2 - x1;
	y2y1 = y2 - y1;
	x2x = x2 - x;
	y2y = y2 - y;
	yy1 = y - y1;
	xx1 = x - x1;
	return 1.0 / (x2x1 * y2y1) * (q11 * x2x * y2y + q21 * xx1 * y2y + q12 * x2x * yy1 + q22 * xx1 * yy1);
}

void SaturnV::GetSICAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const
{
	// CN
	static const int Mach_C_N_arr_size = 13;
	static const int Alpha_C_N_arr_size = 20;
	static const double C_N_arr[Mach_C_N_arr_size][Alpha_C_N_arr_size] = { {0,0.17,0.33,0.5,0.67,0.86,1.05,1.25,1.46,1.65,1.85,2.5,3.25,4.12,5.05,7.29,9.45,11.3,12.42,12.8},
		{0,0.17,0.33,0.5,0.67,0.86,1.05,1.25,1.46,1.65,1.85,2.5,3.25,4.12,5.05,7.29,9.45,11.3,12.42,12.8},
		{0,0.19,0.37,0.55,0.74,0.93,1.14,1.37,1.624,1.88,2.15,2.9,3.85,4.96,6.2,9.25,12.2,14.39,15.79,16.28},
		{0,0.2,0.39,0.58,0.78,1.01,1.23,1.47,1.75,2,2.3,3.25,4.4,5.8,7.35,10.69,14.1,16.6,18.25,18.8},
		{0,0.2,0.39,0.57,0.76,0.95,1.17,1.42,1.69,1.99,2.3,3.3,4.55,6.1,7.85,11.48,15.25,18.15,19.94,20.55},
		{0,0.18,0.36,0.52,0.72,0.93,1.16,1.43,1.71,2,2.3,3.35,4.6,6,7.55,10.69,13.67,16.25,17.88,18.44},
		{0,0.17,0.32,0.5,0.69,0.91,1.17,1.49,1.84,2.25,2.7,3.8,5.05,6.37,7.8,10.5,12.9,14.87,16.31,16.82},
		{0,0.16,0.32,0.49,0.69,0.92,1.19,1.52,1.91,2.4,2.9,4.15,5.35,6.64,7.95,10.46,12.6,14.36,15.68,16.15},
		{0,0.15,0.32,0.48,0.68,0.94,1.23,1.59,2.02,2.49,3,4.15,5.45,6.8,8.05,10.4,12.2,13.72,14.98,15.45},
		{0,0.15,0.31,0.49,0.71,1.01,1.4,1.77,2.19,2.56,3.05,4.05,5.2,6.31,7.5,9.62,11.48,13.02,14.25,14.67},
		{0,0.14,0.31,0.51,0.75,1.03,1.33,1.65,1.99,2.3,2.65,3.63,4.7,5.79,6.95,9.11,11.1,12.74,13.97,14.4},
		{0,0.15,0.31,0.47,0.67,0.89,1.16,1.44,1.76,2.05,2.4,3.3,4.3,5.3,6.4,8.5,10.51,12.36,13.55,13.98},
		{0,0.15,0.31,0.47,0.67,0.89,1.16,1.44,1.77,2.15,2.4,3.3,4.3,5.3,6.4,8.5,10.51,12.25,13.46,13.88} };

	static const double Mach_C_N_arr[Mach_C_N_arr_size] = { 0,0.5,0.8,0.9,1,1.2,1.5,1.7,2,2.75,4,7,8 };
	static const double Alpha_C_N_arr[Alpha_C_N_arr_size] = { 0,2,4,6,8,10,12,14,16,18,20,25,30,35,40,50,60,70,80,90 };

	int i, j;

	for (i = 0; (i < Mach_C_N_arr_size - 2) && (Mach_C_N_arr[i + 1] < Mach); i++);
	for (j = 0; (j < Alpha_C_N_arr_size - 2) && (Alpha_C_N_arr[j + 1] < alpha); j++);

	CN = BilinearInterpolation(C_N_arr[i][j], C_N_arr[i][j + 1], C_N_arr[i + 1][j], C_N_arr[i + 1][j + 1], Mach_C_N_arr[i], Mach_C_N_arr[i + 1], Alpha_C_N_arr[j], Alpha_C_N_arr[j + 1], Mach, alpha);

	//CA
	static const int Mach_C_A_arr_size = 43;
	static const int Alpha_C_A_arr_size = 2;
	static const double C_A_arr[Mach_C_A_arr_size][Alpha_C_A_arr_size] = { {1.15,1.15},
	{1.15,1.15},
	{0.6,0.6},
	{0.39,0.39},
	{0.37,0.37},
	{0.35,0.35},
	{0.34,0.34},
	{0.34,0.34},
	{0.35,0.35},
	{0.37,0.37},
	{0.4,0.4},
	{0.45,0.45},
	{0.52,0.52},
	{0.6,0.6},
	{0.7,0.7},
	{0.75,0.75},
	{0.78,0.78},
	{0.8,0.8},
	{0.82,0.82},
	{0.82,0.82},
	{0.8,0.8},
	{0.78,0.78},
	{0.75,0.75},
	{0.69,0.69},
	{0.65,0.65},
	{0.6,0.6},
	{0.54,0.54},
	{0.5,0.5},
	{0.45,0.45},
	{0.4,0.4},
	{0.33,0.33},
	{0.27,0.27},
	{0.2,0.2},
	{0.15,0.15},
	{0.12,0.12},
	{0.08,0.08},
	{0.04,0.04},
	{0,0},
	{-0.05,-0.05},
	{-0.08,-0.08},
	{-0.095,-0.095},
	{0.175,0.175},
	{0.16,0.16} };

	static const double Mach_C_A_arr[Mach_C_A_arr_size] = { 0,0.1,0.15,0.2,0.22,0.25,0.3,0.35,0.43,0.51,0.59,0.7,0.8,0.9,0.98,1.02,1.06,
		1.1,1.15,1.2,1.25,1.3,1.35,1.45,1.53,1.67,1.85,2,2.22,2.5,
		3.02,3.52,4.3,5.07,5.55,6.08,6.55,6.9,7.28,7.5,7.5512,7.615,8 };
	static const double Alpha_C_A_arr[Alpha_C_A_arr_size] = {0,50};

	for (i = 0; (i < Mach_C_A_arr_size - 2) && (Mach_C_A_arr[i + 1] < Mach); i++);
	for (j = 0; (j < Alpha_C_A_arr_size - 2) && (Alpha_C_A_arr[j + 1] < alpha); j++);

	CA = BilinearInterpolation(C_A_arr[i][j], C_A_arr[i][j + 1], C_A_arr[i + 1][j], C_A_arr[i + 1][j + 1], Mach_C_A_arr[i], Mach_C_A_arr[i + 1], Alpha_C_A_arr[j], Alpha_C_A_arr[j + 1], Mach, alpha);

	// Center of pressure
	static const int Mach_COP_arr_size = 13;
	static const int Alpha_COP_arr_size = 20;

	double COP_arr[Mach_COP_arr_size][Alpha_COP_arr_size] = { {3.31,3.41,3.47,3.51,3.6,3.69,3.81,3.9,3.97,3.78,3.95,3.9,3.9,3.9,3.9,3.8,3.68,3.53,3.45,3.38},
	{3.31,3.41,3.47,3.51,3.6,3.69,3.81,3.9,3.97,3.78,3.95,3.9,3.9,3.9,3.9,3.8,3.68,3.53,3.45,3.38},
	{2.98,3.02,3.08,3.2,3.32,3.45,3.61,3.7,3.78,3.82,3.86,3.89,3.9,3.9,3.9,3.85,3.74,3.63,3.58,3.55},
	{2.81,2.81,2.88,3.04,3.18,3.34,3.46,3.61,3.7,3.76,3.79,3.85,3.89,3.9,3.9,3.87,3.79,3.68,3.62,3.6},
	{2.86,2.94,3.04,3.17,3.37,3.58,3.68,3.84,3.93,3.98,4,4.04,4.08,4.09,4.1,4.01,3.85,3.73,3.68,3.65},
	{3.3,3.35,3.43,3.54,3.73,3.84,3.94,4.04,4.08,4.12,4.15,4.18,4.19,4.18,4.18,4.03,3.9,3.8,3.79,3.74},
	{3.92,3.92,3.96,4,4.07,4.11,4.17,4.2,4.22,4.25,4.25,4.24,4.2,4.15,4.07,3.99,3.91,3.88,3.87,3.85},
	{4.13,4.13,4.16,4.19,4.24,4.25,4.25,4.26,4.26,4.26,4.25,4.22,4.18,4.11,4.03,3.94,3.9,3.9,3.9,3.9},
	{4.32,4.32,4.32,4.3,4.3,4.3,4.3,4.3,4.29,4.28,4.26,4.2,4.16,4.08,4,3.92,3.9,3.9,3.9,3.9},
	{4.45,4.45,4.4,4.35,4.34,4.32,4.31,4.3,4.28,4.28,4.22,4.15,4.1,4.05,4,3.94,3.9,3.9,3.9,3.9},
	{4.45,4.44,4.41,4.38,4.37,4.33,4.32,4.31,4.3,4.28,4.25,4.18,4.1,4.04,4,3.91,3.9,3.9,3.9,3.9},
	{5.08,5,4.95,4.72,4.4,4.35,4.34,4.33,4.3,4.27,4.25,4.16,4.11,4.04,4,3.93,3.9,3.9,3.9,3.9},
	{5.08,5,4.95,4.72,4.4,4.35,4.34,4.33,4.3,4.27,4.25,4.16,4.11,4.04,4,3.93,3.9,3.9,3.9,3.9} };

	static const double Mach_COP_arr[Mach_COP_arr_size] = { 0,0.5,0.8,0.9,1,1.2,1.5,1.7,2,2.75,4,7,8 };
	static const double Alpha_COP_arr[Alpha_COP_arr_size] = { 0,2,4,6,8,10,12,14,16,18,20,25,30,35,40,50,60,70,80,90 };

	for (i = 0; (i < Mach_COP_arr_size - 2) && (Mach_COP_arr[i + 1] < Mach); i++);
	for (j = 0; (j < Alpha_COP_arr_size - 2) && (Alpha_COP_arr[j + 1] < alpha); j++);

	CP = BilinearInterpolation(COP_arr[i][j], COP_arr[i][j + 1], COP_arr[i + 1][j], COP_arr[i + 1][j + 1], Mach_COP_arr[i], Mach_COP_arr[i + 1], Alpha_COP_arr[j], Alpha_COP_arr[j + 1], Mach, alpha);
	
	// This CP is in calibers relative to the S-IC gimbal plane. Convert to meters and body coordinates.
	CP = CP * 33.0 * MathConstants::FOOT_TO_M + 100.0 * 0.0254;

	// Area
	Area = (855.0 * 0.3048 * 0.3048);
}

void SaturnV::GetSIIAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const
{
	// CN

	static const int Mach_C_N_arr_size = 5;
	static const int Alpha_C_N_arr_size = 9;

	static const double C_N_arr[Mach_C_N_arr_size][Alpha_C_N_arr_size] = { {0.004,0.088,0.184,0.3,0.435,0.575,0.73,0.9,1.075},
	{0,0.096,0.194,0.305,0.43,0.565,0.715,0.88,1.06},
	{0,0.104,0.192,0.29,0.4,0.53,0.68,0.85,1.02},
	{0,0.124,0.214,0.3,0.42,0.55,0.7,0.86,1.045},
	{0,0.1,0.2,0.31,0.42,0.55,0.71,0.87,1.03} };

	static const double Mach_C_N_arr[Mach_C_N_arr_size] = { 5.02,6.06,7.06,7.87,10 };
	static const double Alpha_C_N_arr[Alpha_C_N_arr_size] = { 0,2,4,6,8,10,12,14,16 };

	int i, j;

	for (i = 0; (i < Mach_C_N_arr_size - 2) && (Mach_C_N_arr[i + 1] < Mach); i++);
	for (j = 0; (j < Alpha_C_N_arr_size - 2) && (Alpha_C_N_arr[j + 1] < alpha); j++);

	CN = BilinearInterpolation(C_N_arr[i][j], C_N_arr[i][j + 1], C_N_arr[i + 1][j], C_N_arr[i + 1][j + 1], Mach_C_N_arr[i], Mach_C_N_arr[i + 1], Alpha_C_N_arr[j], Alpha_C_N_arr[j + 1], Mach, alpha);

	// CA
	// S-II CA

	static const int Mach_C_A_arr_size = 4;
	static const int Alpha_C_A_arr_size = 11;

	static const double C_A_arr[Mach_C_A_arr_size][Alpha_C_A_arr_size] = { {0.15,0.155,0.16,0.162,0.17,0.185,0.208,0.23,0.255,0.28,0.31},
	{0.098,0.12,0.162,0.185,0.186,0.206,0.23,0.255,0.281,0.31,0.34},
	{0.095,0.119,0.163,0.187,0.19,0.213,0.235,0.262,0.29,0.32,0.35},
	{0.095,0.119,0.163,0.187,0.19,0.213,0.235,0.262,0.29,0.32,0.35} };

	static const double Mach_C_A_arr[Mach_C_A_arr_size] = { 7,8,9,10 };
	static const double Alpha_C_A_arr[Alpha_C_A_arr_size] = { 0,1,2,3,4,6,8,10,12,14,16 };

	for (i = 0; (i < Mach_C_A_arr_size - 2) && (Mach_C_A_arr[i + 1] < Mach); i++);
	for (j = 0; (j < Alpha_C_A_arr_size - 2) && (Alpha_C_A_arr[j + 1] < alpha); j++);

	CA = BilinearInterpolation(C_A_arr[i][j], C_A_arr[i][j + 1], C_A_arr[i + 1][j], C_A_arr[i + 1][j + 1], Mach_C_A_arr[i], Mach_C_A_arr[i + 1], Alpha_C_A_arr[j], Alpha_C_A_arr[j + 1], Mach, alpha);

	// CP
	static const int Mach_COP_arr_size = 5;
	static const int Alpha_COP_arr_size = 10;

	double COP_arr[Mach_COP_arr_size][Alpha_COP_arr_size] = { {2.75,2.88,2.87,2.85,2.8,2.72,2.62,2.57,2.55,2.53},
	{3.2,3.44,3.37,3.18,2.9,2.79,2.69,2.65,2.59,2.56},
	{2.2,3.34,3.41,3.32,3.03,2.84,2.7,2.68,2.59,2.59},
	{3.2,3.16,3.35,3.32,3.09,2.89,2.7,2.56,2.63,2.6},
	{3.4,3.39,3.36,3.32,3.07,2.87,2.76,2.71,2.67,2.63} };

	static const double Mach_COP_arr[Mach_COP_arr_size] = { 5.02,6.06,7.06,7.87,10 };
	static const double Alpha_COP_arr[Alpha_COP_arr_size] = { 0,2,3,4,6,8,10,12,14,16 };

	CP = BilinearInterpolation(COP_arr[i][j], COP_arr[i][j + 1], COP_arr[i + 1][j], COP_arr[i + 1][j + 1], Mach_COP_arr[i], Mach_COP_arr[i + 1], Alpha_COP_arr[j], Alpha_COP_arr[j + 1], Mach, alpha);

	// This CP is in calibers relative to the S-II gimbal plane. Convert to meters and body coordinates.
	CP = CP * 33.0 * MathConstants::FOOT_TO_M + 130.333*0.3048 + 100.0 * 0.0254;

	Area = (855.0 * 0.3048 * 0.3048);
}

void SaturnV::GetSIVBAerodynamicCoefficients(double alpha, double Mach, double& CN, double& CA, double& CP, double& Area) const
{
	CN = 0.0;
	CA = 2.0;
	CP = 0.0;
	Area = 370.0 * 0.3048 * 0.3048;
}

IU* SaturnV::GetIU()
{
	return IU;
}