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

#include "j2engine.h"
#include "Saturn.h"
#include "../util/Constants.h"

J2Engine::J2Engine(Saturn* v, ThrustSpec* j2)
	:vessel(v), th_j2(j2)
{
	EngineReady = false;
	EngineReadyBypass = false;
	EngineStart = false;
	EngineStop = false;
	LVDCEngineStopRelay = false;
	EDSCutoff = false;
	RSSCutoff = false;
	EngineRunning = false;
	ThrustOK = false;
	EngineFailed = false;

	ThrustTimer = 0.0;
	ThrustLevel = 0.0;
}

void J2Engine::Timestep(double simdt)
{
	ThrustOK = vessel->GetThrusterLevel(th_j2) > 0.65;

	if (!EngineRunning)
	{
		EngineReady = true;
	}
	else
	{
		EngineReady = false;
	}

	if ((EngineReady || EngineReadyBypass) && EngineStart)
	{
		EngineStart = true;
	}
	else
	{
		EngineStart = false;
	}

	if (LVDCEngineStopRelay || EDSCutoff || RSSCutoff || (!ThrustOK && EngineRunning))
	{
		EngineStop = true;
	}
	else
	{
		EngineStop = false;
	}

	if (EngineStop || EngineFailed)
	{
		if (ThrustLevel > 0.0)
		{
			ThrustTimer += simdt;

			// Cutoff transient thrust
			if (ThrustTimer < 0.25) {
				// 95% of thrust dies in the first .25 second
				ThrustLevel = 1.0 - (ThrustTimer * 3.3048);
				vessel->SetThrusterLevel(th_j2, ThrustLevel);
			}
			else {
				if (ThrustTimer < 1.5) {
					// The remainder dies over the next 1.25 second
					ThrustLevel = 0.1738 - ((ThrustTimer - 0.25) * 0.1390);
					vessel->SetThrusterLevel(th_j2, ThrustLevel);
				}
				else {
					// Engine is completely shut down at 1.5 second
					ThrustLevel = 0.0;
					vessel->SetThrusterLevel(th_j2, ThrustLevel);
					EngineRunning = false;
					ThrustTimer = 0.0;
				}
			}
		}
	}
	else if (EngineStart && !EngineRunning)
	{
		ThrustTimer += simdt;

		if (ThrustTimer < 1.0)
		{
			ThrustLevel = 0.0;
		}
		else if (ThrustTimer < 3.2)
		{
			ThrustLevel = (ThrustTimer - 1.0) * 0.45;
		}
		else
		{
			ThrustLevel = 1.0;
			EngineRunning = true;
			ThrustTimer = 0.0;
		}
		vessel->SetThrusterLevel(th_j2, ThrustLevel);
	}
	else if (ThrustTimer > 0.0)
	{
		ThrustTimer = 0.0;
	}

	/*
	if (EngineFailed)
	{
		sprintf(oapiDebugString(), "EngineStart %d EngineStop %d EngineRunning %d ThrustTimer %lf ThrustLevel %lf", EngineStart, EngineStop, EngineRunning, ThrustTimer, ThrustLevel);
	}
	*/
}

void J2Engine::SetThrusterIsp(double isp)
{
	vessel->SetThrusterIsp(th_j2, isp);
}

void J2Engine::SetThrusterMax0(double thrust)
{
	vessel->SetThrusterMax0(th_j2, thrust);
}

void J2Engine::SetThrusterDir(double beta_y, double beta_p)
{
	const double gimb_limit = 7.0 * MathConstants::RAD_PER_DEG;

	if (beta_y > gimb_limit)
	{
		beta_y = gimb_limit;
	}
	else if (beta_y < -gimb_limit)
	{
		beta_y = -gimb_limit;
	}

	if (beta_p > gimb_limit)
	{
		beta_p = gimb_limit;
	}
	else if (beta_p < -gimb_limit)
	{
		beta_p = -gimb_limit;
	}

	vessel->SetThrusterDir(th_j2, beta_y, beta_p);
}