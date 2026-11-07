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

#include "s1csystems.h"
#include "../Saturn.h"
#include "../../util/Constants.h"
#include <algorithm>
#include "../connector_messages.h"

SICSystemsConnector::SICSystemsConnector()
{
	ourSIC = NULL;
}

SICSystemsConnector::~SICSystemsConnector()
{

}

SICToSIIConnector::SICToSIIConnector()
{
	type = SII_IS_SIC_COMMAND;
}

SICToSIIConnector::~SICToSIIConnector()
{

}

bool SICToSIIConnector::ReceiveMessage(Connector* from, ConnectorMessage& m)
{
	//
	// Sanity check.
	//

	if (m.destination != type)
	{
		return false;
	}

	SIISICMessageType messageType;

	messageType = (SIISICMessageType)m.messageType;

	switch (messageType)
	{
	case SII_SIC_SI_THRUSTER_DIR:
		if (ourSIC)
		{
			ourSIC->SetThrusterDir(m.val1.iValue, m.val2.dValue, m.val3.dValue);
			return true;
		}
		break;
	case SII_SIC_SI_EDS_CUTOFF:
		if (ourSIC)
		{
			ourSIC->EDSEnginesCutoff(m.val1.bValue);
			return true;
		}
		break;
	case SII_SIC_SI_SWITCH_SELECTOR:
		if (ourSIC)
		{
			ourSIC->GetSwitchSelector()->ProcessCommand(m.val1.iValue);
			return true;
		}
		break;
	case SII_SIC_SWITCH_SELECTOR_FEEDBACK:
		if (ourSIC)
		{
			return ourSIC->GetSwitchSelector()->GetFeedback();
		}
		break;
	case SII_SIC_GETSITHRUSTOK:
		if (ourSIC)
		{
			ourSIC->GetThrustOK((bool*)m.val1.pValue);
			return true;
		}
		break;
	case SII_SIC_PROPELLANT_DEPLETION_ENGINE_CUTOFF:
		if (ourSIC)
		{
			m.val1.bValue = ourSIC->GetPropellantDepletionEngineCutoff();
			return true;
		}
		break;
	case SII_SIC_GET_SI_INBOARD_ENGINE_OUT:
		if (ourSIC)
		{
			m.val1.bValue = ourSIC->GetInboardEngineOut();
			return true;
		}
		break;
	case SII_SIC_GET_SI_OUTBOARD_ENGINE_OUT:
		if (ourSIC)
		{
			m.val1.bValue = ourSIC->GetOutboardEngineOut();
			return true;
		}
		break;
	case SII_SIC_GET_SIC_SII_NOT_SEPARATED:
		if (ourSIC)
		{
			m.val1.bValue = ourSIC->GetSICSIINotSeparated();
			return true;
		}
		break;
	}
	return false;
}

SICToSIESEConnector::SICToSIESEConnector()
{
	type = SIESE_SI_COMMAND;
}

SICToSIESEConnector::~SICToSIESEConnector()
{

}

bool SICToSIESEConnector::GetSICThrustOKSimulate(int eng, int n)
{
	ConnectorMessage m;

	m.destination = SIESE_SI_COMMAND;
	m.messageType = SI_SIESE_GET_SI_THRUST_OK_SIMULATE;
	m.val1.iValue = eng;
	m.val2.iValue = n;

	if (SendMessage(m))
	{
		return m.val1.bValue;
	}

	return false;
}

bool SICToSIESEConnector::ReceiveMessage(Connector* from, ConnectorMessage& m)
{
	//
	// Sanity check.
	//

	if (m.destination != type)
	{
		return false;
	}

	SIESEMessageType messageType;

	messageType = (SIESEMessageType)m.messageType;

	switch (messageType)
	{
	case SIESE_SI_SI_STAGE_LOGIC_CUTOFF:
		if (ourSIC)
		{
			m.val1.bValue = ourSIC->GetEngineStop();
			return true;
		}
		break;
	case SIESE_SI_SET_ENGINE_START:
		if (ourSIC)
		{
			ourSIC->SetEngineStart(m.val1.iValue);
			return true;
		}
		break;
	case SIESE_SI_GSE_CUTOFF:
		if (ourSIC)
		{
			ourSIC->GSEEnginesCutoff(m.val1.bValue);
			return true;
		}
		break;
	case SIESE_SI_THRUST_OK:
		if (ourSIC)
		{
			ourSIC->GetThrustOK((bool*)m.val1.pValue);
			return true;
		}
		break;
	}
	return false;
}

F1Engine::F1Engine(Saturn* v, ThrustSpec* f1)
	:vessel(v), th_f1(f1)
{
	EngineStart = false;
	EngineStop = false;
	ProgrammedCutoff = false;
	EDSCutoff = false;
	GSECutoff = false;
	RSSCutoff = false;
	EngineRunning = false;
	ThrustOK = false;
	EngineFailed = false;

	ThrustTimer = 0.0;
	ThrustLevel = 0.0;

	pitchPos = 0.0;
	yawPos = 0.0;
}

void F1Engine::Timestep(double simdt)
{
	if (th_f1 == NULL) return;

	//Thrust OK switch
	ThrustOK = vessel->GetThrusterLevel(th_f1) > 0.9 && !EngineFailed;

	if (ProgrammedCutoff || EDSCutoff || GSECutoff || RSSCutoff)
	{
		EngineStop = true;
	}

	if (EngineStop)
	{
		if (ThrustLevel > 0.0)
		{
			EngineStart = false;
			EngineRunning = false;

			double tm_1, tm_2, tm_3;

			ThrustTimer += simdt;

			tm_1 = 0.1;
			tm_2 = 0.3;
			tm_3 = 0.7;

			if (ThrustTimer >= tm_1)
			{
				if (ThrustTimer < tm_2)
				{
					ThrustLevel = std::min(ThrustLevel, 1.0 - 3.35 * (ThrustTimer - tm_1));
				}
				else
				{
					if (ThrustTimer < tm_3)
					{
						ThrustLevel = std::min(ThrustLevel, 0.33 - 0.825 * (ThrustTimer - tm_2));
					}
					else
					{
						ThrustLevel = 0;
					}
				}
			}
			else
			{
				ThrustLevel = std::min(ThrustLevel, 1.0);
			}

			vessel->SetThrusterLevel(th_f1, ThrustLevel);
		}
	}
	else if (EngineStart && !EngineRunning)
	{
		// SATURN V ENGINE STARTUP
		// Engine startup was staggered 1-2-2, with engine 5 starting first, then 1+3, then 2+4. 
		// This happened by the starter solenoid operating at T-6.585 for engine 5.

		// Engine 5 combustion chamber ignition was at T-3.315, engines 1+3 at T-3.035, and engines 2+4 at T-2.615
		// The engines idled in low-range thrust (about 2.5% thrust) for about 0.3 seconds
		// and then rose to 93% thrust in 0.85 seconds.
		// The rise from 93 to 100 percent thrust took 0.75 second.
		// Total engine startup time was 1.9 seconds.

		// Source: Apollo 8 LV Flight Evaluation

		double tm_1, tm_2, tm_3, tm_4;

		ThrustTimer += simdt;

		tm_1 = -3.315 + 8.9;
		tm_2 = tm_1 + 0.3;  // Start of 1st rise
		tm_3 = tm_2 + 0.85; // Start of 2nd rise
		tm_4 = tm_3 + 0.75; // End of 2nd rise
		if (ThrustTimer >= tm_1) {
			// Light CC
			if (ThrustTimer < tm_2) {
				// Idle at 2.5% thrust
				ThrustLevel = 0.025;
			}
			else {
				if (ThrustTimer < tm_3) {
					// Rise to 93% at a rate of 106 percent per second
					ThrustLevel = 0.025 + (1.06 * (ThrustTimer - tm_2));
				}
				else {
					if (ThrustTimer < tm_4) {
						// Rise to 100% at a rate of 9 percent per second.
						ThrustLevel = 0.93 + (0.09 * (ThrustTimer - tm_3));
					}
					else {
						// Hold 100%
						ThrustLevel = 1;
						EngineRunning = true;
					}
				}
			}
		}
		else
		{
			ThrustLevel = 0.0;
		}

		vessel->SetThrusterLevel(th_f1, ThrustLevel);
	}
	else if (ThrustTimer > 0.0)
	{
		ThrustTimer = 0.0;
	}
}

void F1Engine::SetThrusterDir(double beta_y, double beta_p)
{
	const double gimb_limit = 5.16 * MathConstants::RAD_PER_DEG;

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

	yawPos = beta_y;
	pitchPos = beta_p;

	vessel->SetThrusterDir(th_f1, beta_y, beta_p);
}

SICSystems::SICSystems(Saturn* s, ThrustSpec* f1, TankSpec* f1prop) : StageSystems(s),
	ourSwitchSelector(01, this),
	main_propellant(f1prop),
	f1engine1(s, &f1[0]),
	f1engine2(s, &f1[1]),
	f1engine3(s, &f1[2]),
	f1engine4(s, &f1[3]),
	f1engine5(s, &f1[4])
{
	for (int i = 0; i < 15; i++)
	{
		ThrustOK[i] = false;
	}

	MultipleEngineCutoffEnabled = false;
	PropellantDepletionSensors = false;
	PointLevelSensorArmed = false;
	TwoAdjacentOutboardEnginesOutCutoff = false;

	f1engines[0] = &f1engine1;
	f1engines[1] = &f1engine2;
	f1engines[2] = &f1engine3;
	f1engines[3] = &f1engine4;
	f1engines[4] = &f1engine5;

	sicSIIConnector.SetSICSystems(this);
	sicSIESEConnector.SetSICSystems(this);
}

SICSystems::~SICSystems()
{
	sicSIIConnector.Disconnect();
	sicSIESEConnector.Disconnect();
}

void SICSystems::Timestep(double misst, double simdt)
{
	f1engine1.Timestep(simdt);
	f1engine2.Timestep(simdt);
	f1engine3.Timestep(simdt);
	f1engine4.Timestep(simdt);
	f1engine5.Timestep(simdt);

	//Thrust OK
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ThrustOK[i * 3 + j] = f1engines[i]->GetThrustOK() || sicSIESEConnector.GetSICThrustOKSimulate(i + 1, j + 1);
		}
	}

	//Propellant Depletion
	if (PropellantLowLevel())
	{
		PropellantDepletionSensors = true;
	}
	else
	{
		PropellantDepletionSensors = false;
	}

	if (PropellantDepletionSensors)
	{
		OutboardEnginesCutoff();
	}

	if (TwoAdjacentOutboardEnginesOutCutoff)
	{
		if ((!f1engine1.GetThrustOK() && !f1engine2.GetThrustOK()) ||
			(!f1engine2.GetThrustOK() && !f1engine3.GetThrustOK()) ||
			(!f1engine3.GetThrustOK() && !f1engine4.GetThrustOK()) ||
			(!f1engine4.GetThrustOK() && !f1engine1.GetThrustOK()))
		{
			OutboardEnginesCutoff();
		}
	}

	if (MultipleEngineCutoffEnabled)
	{
		for (int i = 0; i < 5; i++)
		{
			if (f1engines[i]->GetThrustOK() == false) f1engines[i]->SetProgrammedEngineCutoff();
		}
	}

	//sprintf(oapiDebugString(), "%f %f %f %f %f %f", contrailLevel, f1engine1.GetThrustLevel(), f1engine2.GetThrustLevel(), f1engine3.GetThrustLevel(), f1engine4.GetThrustLevel(), f1engine5.GetThrustLevel());
	//sprintf(oapiDebugString(), "%f %f %f %f %f", FirstStageFailureTime[0], FirstStageFailureTime[1], FirstStageFailureTime[2], FirstStageFailureTime[3], FirstStageFailureTime[4]);
}

bool SICSystems::PropellantLowLevel()
{
	if (PointLevelSensorArmed)
	{
		if (main_propellant)
		{
			if (GetSat()->GetPropellantMass(main_propellant) < 30000.0)
			{
				return true;
			}
		}
	}

	return false;
}

void SICSystems::InboardEngineCutoff()
{
	f1engine5.SetProgrammedEngineCutoff();
}

void SICSystems::OutboardEnginesCutoff()
{
	f1engine1.SetProgrammedEngineCutoff();
	f1engine2.SetProgrammedEngineCutoff();
	f1engine3.SetProgrammedEngineCutoff();
	f1engine4.SetProgrammedEngineCutoff();
}

void SICSystems::SetEngineStart(int n)
{
	if (n < 1 || n > 5) return;

	f1engines[n - 1]->SetEngineStart();
}

void SICSystems::SetThrusterDir(int n, double beta_y, double beta_p)
{
	if (n < 0 || n > 3) return;

	f1engines[n]->SetThrusterDir(beta_y, beta_p);
}

void SICSystems::EDSEnginesCutoff(bool cut)
{
	if (cut)
	{
		f1engine1.SetEDSCutoff();
		f1engine2.SetEDSCutoff();
		f1engine3.SetEDSCutoff();
		f1engine4.SetEDSCutoff();
		f1engine5.SetEDSCutoff();
	}
}

void SICSystems::GSEEnginesCutoff(bool cut)
{
	if (cut)
	{
		f1engine1.SetGSECutoff();
		f1engine2.SetGSECutoff();
		f1engine3.SetGSECutoff();
		f1engine4.SetGSECutoff();
		f1engine5.SetGSECutoff();
	}
}

void SICSystems::GetThrustOK(bool* ok)
{
	for (int i = 0; i < 15; i++)
	{
		ok[i] = ThrustOK[i];
	}
}

void SICSystems::SetEngineFailed(int n, bool condition)
{
	if (n >= 0 && n <= 4)
	{
		f1engines[n]->SetFailed(condition);
	}
}

double SICSystems::GetSumThrust()
{
	return (f1engine1.GetThrustLevel() + f1engine2.GetThrustLevel() + f1engine3.GetThrustLevel() + f1engine4.GetThrustLevel() + f1engine5.GetThrustLevel()) / 5.0;
}

bool SICSystems::TripleVoting(bool vote1, bool vote2, bool vote3)
{
	int num = 0;
	if (vote1) num++;
	if (vote2) num++;
	if (vote3) num++;
	if (num >= 2) return true;
	return false;
}

void SICSystems::SwitchSelectorOutput(int channel)
{
	GetSat()->SendEventMessage("S-IC Switch Selector Channel " + std::to_string(channel));

	switch (channel)
	{
	case 0: //Liftoff (NOT A REAL SWITCH SELECTOR EVENT)
		break;
	case 1: //Telemeter Calibrate Off
		break;
	case 2: //Telemeter Calibrate On
		break;
	case 3: //Multiple Engine Cutoff Enable
		MultipleEngineCutoffEnable();
		break;
	case 4: //LOX Tank Strobe Lights Off
		break;
	case 5: //Fuel Pressurizing Valve No. 2 Open & Tape Recorder Record
		break;
	case 6: //Fuel Pressurizing Valve No. 3 Open
		break;
	case 7: //Fuel Pressurizing Valve No. 4 Open
		break;
	case 8: //Inboard Engine Cutoff
		InboardEngineCutoff();
		break;
	case 9: //Outboard Engines Cutoff Enable
		OutboardEnginesCutoffEnable();
		break;
	case 10: //Separation and Retro No. 1 EBW Firing Units Arm
		break;
	case 12: //Separation Camera On
		break;
	case 13: //Telemetry Measurement Switchover
		break;
	case 14: //Outboard Engines Cutoff Backup Enable
		OutboardEnginesCutoffEnable();
		break;
	case 15: //S-IC/S-II Separation (No. 1)
		SIC_SII_Separation_Pyros.SetBlown(true);
		break;
	case 16: //Inboard Engine Cutoff Backup
		InboardEngineCutoff();
		break;
	case 17: //Two Adjacent Outboard Engines Out Cutoff Enable
		TwoAdjacentOutboardEnginesOutCutoffEnable();
		break;
	case 19: //S-IC/S-II Separation (No. 2)
		// TBD: SIC_SII_Separation_Pyros.SetBlown(true);
		break;
	case 20: //Separation and Retro No. 2 EBW Firing Units Arm
		break;
	default:
		break;
	}
}

bool SICSystems::GetPropellantDepletionEngineCutoff()
{
	if (PointLevelSensorArmed)
	{
		if (PropellantDepletionSensors) return true;
		if (!TripleVoting(ThrustOK[0], ThrustOK[1], ThrustOK[2]) && !TripleVoting(ThrustOK[3], ThrustOK[4], ThrustOK[5]) && !TripleVoting(ThrustOK[6], ThrustOK[7], ThrustOK[8]) && !TripleVoting(ThrustOK[9], ThrustOK[10], ThrustOK[11])) return true;
	}

	return false;
}

bool SICSystems::GetInboardEngineOut()
{
	// TBD: Is this right?
	if (MultipleEngineCutoffEnabled == false) return false;

	if (!TripleVoting(ThrustOK[12], ThrustOK[13], ThrustOK[14])) return true;

	return false;
}

bool SICSystems::GetOutboardEngineOut()
{
	for (int i = 0; i < 4; i++) if (!TripleVoting(ThrustOK[3 * i], ThrustOK[3 * i + 1], ThrustOK[3 * i + 2])) return true;

	return false;
}

bool SICSystems::GetSICSIINotSeparated()
{
	//TBD: GSE S-IC/S-II sep simulate
	return true;
}

bool SICSystems::IsPyroBlown()
{
	return SIC_SII_Separation_Pyros.IsBlown();
}

bool SICSystems::GetEngineStop()
{
	for (int i = 0; i < 5; i++) if (f1engines[i]->GetEngineStop()) return true;

	return false;
}