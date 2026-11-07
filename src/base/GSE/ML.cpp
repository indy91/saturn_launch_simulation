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

#include "ML.h"
#include "SIC_ESE.h"
#include "IU_ESE.h"
#include "../Saturn.h"

ML::ML(Saturn* s) : sat(s), rca110A(90.0)
{
	EventCounter = 0;
	SIESE = NULL;
	IUESE = NULL;
}

ML::~ML()
{
	if (SIESE)
	{
		delete SIESE;
		SIESE = NULL;
	}
	if (IUESE)
	{
		delete IUESE;
		IUESE = NULL;
	}
}

void ML::Init(double gmtlo, bool IsSaturnV)
{
	EventCounter = 0;

	// Create
	if (IsSaturnV)
	{
		SIESE = new SIC_ESE();
		IUESE = new IUSV_ESE();
	}

	// Wire together
	IUESE->Init(&rca110A);
	rca110A.Init(SIESE, IUESE, gmtlo);
}

void ML::PreStep(double t, double simdt)
{
	double t_count = t - rca110A.GetGMTLO();

	rca110A.Timestep(t, simdt);
	IUESE->Timestep(t_count, simdt);

	switch (EventCounter)
	{
	case 0: // Turn LVDC power on
		if (t_count >= -620.0)
		{
			GetSat()->SendEventMessage("FT04 - LVDC Power On");
			rca110A.StartFT04LVDCPowerUp();
			EventCounter++;
		}
		break;
	case 1: // Azimuth laying
		if (t_count >= -610.0)
		{
			GetSat()->SendEventMessage("GT16 - Azimuth Laying");
			rca110A.StartGT16AzimuthLaying(72.0);
			EventCounter++;
		}
		break;
	case 2:
		if (t_count >= -600.0)
		{
			rca110A.StartFT05DataLoadTest();
			GetSat()->SendEventMessage("FT05 - Load Test");
			//rca110A.StartFT42PrepareToLaunch();
			//GetSat()->SendEventMessage("FT42 - Prepare To Launch");
			EventCounter++;
		}
		else break;
	case 3:
		if (t_count >= -22.0)
		{
			IUESE->SetGuidanceReferenceReleaseAlert(true);
			GetSat()->SendEventMessage("GRR Alert");
			EventCounter++;
		}
		else break;
	case 4:
		if (t_count >= -17.0)
		{
			IUESE->SetGuidanceReferenceRelease(true);
			GetSat()->SendEventMessage("GRR");
			EventCounter++;
		}
		else break;
	case 5:
		if (t_count >= -8.9)
		{
			GetSat()->SendEventMessage("Engine 5 Start");
			SIESE->GetSIESEToSICommandConnector()->SetEngineStart(5);
			EventCounter++;
		}
		else break;
	case 6:
		if (t_count >= -8.62)
		{
			GetSat()->SendEventMessage("Engine 1 Start");
			GetSat()->SendEventMessage("Engine 3 Start");
			SIESE->GetSIESEToSICommandConnector()->SetEngineStart(1);
			SIESE->GetSIESEToSICommandConnector()->SetEngineStart(3);
			EventCounter++;
		}
		else break;
	case 7:
		if (t_count >= -8.2)
		{
			GetSat()->SendEventMessage("Engine 2 Start");
			GetSat()->SendEventMessage("Engine 4 Start");
			SIESE->GetSIESEToSICommandConnector()->SetEngineStart(2);
			SIESE->GetSIESEToSICommandConnector()->SetEngineStart(4);
			EventCounter++;
		}
		else break;
	}
}

void ML::PostStep(double t, double simdt)
{

}

void ML::ConnectSIUmbilical(Connector* c)
{
	SIESE->GetSIESEToSICommandConnector()->ConnectTo(c);
}

void ML::ConnectIUUmbilical(Connector* c)
{
	IUESE->GetIUESEToIUCommandConnector()->ConnectTo(c);
}

void ML::DisconnectUmbilicals()
{
	SIESE->GetSIESEToSICommandConnector()->Disconnect();
	IUESE->GetIUESEToIUCommandConnector()->Disconnect();
}

bool ML::CutoffInterlock()
{
	return (IUESE->GetIUESEToIUCommandConnector()->IsEDSUnsafeA() || IUESE->GetIUESEToIUCommandConnector()->IsEDSUnsafeB() || SIESE->GetSIESEToSICommandConnector()->SIStageLogicCutoff());
}

bool ML::Commit()
{
	return IUESE->GetIUESEToIUCommandConnector()->AllSIEnginesRunning() && !CutoffInterlock();
}

Saturn* ML::GetSat()
{
	return sat;
}