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

#include "../Saturn.h"
#include "siisystems.h"
#include "../connector_messages.h"

const double G = 9.81;

SIISystemsConnector::SIISystemsConnector()
{
	ourSII = NULL;
}

SIISystemsConnector::~SIISystemsConnector()
{

}

SIIToSIVBConnector::SIIToSIVBConnector()
{
	type = SIVB_SIX_COMMAND;
}

SIIToSIVBConnector::~SIIToSIVBConnector()
{

}

bool SIIToSIVBConnector::ReceiveMessage(Connector* from, ConnectorMessage& m)
{
	//
	// Sanity check.
	//

	if (m.destination != type)
	{
		return false;
	}

	SIVBSIXMessageType messageType;

	messageType = (SIVBSIXMessageType)m.messageType;

	switch (messageType)
	{
	case SIVB_SIX_SI_THRUSTER_DIR:
		if (ourSII)
		{
			ourSII->GetSIIToSIIInterstageConnector()->SetSIThrusterDir(m.val1.iValue, m.val2.dValue, m.val3.dValue);
			return true;
		}
		break;
	case SIVB_SIX_SII_THRUSTER_DIR:
		if (ourSII)
		{
			ourSII->SetThrusterDir(m.val1.iValue, m.val2.dValue, m.val3.dValue);
			return true;
		}
		break;
	case SIVB_SIX_SI_EDS_CUTOFF:
		if (ourSII)
		{
			ourSII->GetSIIToSIIInterstageConnector()->SIEDSCutoff(m.val1.bValue);
			return true;
		}
		break;
	case SIVB_SIX_SII_EDS_CUTOFF:
		if (ourSII)
		{
			ourSII->EDSEnginesCutoff(m.val1.bValue);
			return true;
		}
		break;
	case SIVB_SIX_SI_SWITCH_SELECTOR:
		if (ourSII)
		{
			ourSII->GetSIIToSIIInterstageConnector()->SISwitchSelector(m.val1.iValue);
			return true;
		}
		break;
	case SIVB_SIX_SII_SWITCH_SELECTOR:
		if (ourSII)
		{
			ourSII->GetSwitchSelector()->ProcessCommand(m.val1.iValue);
			return true;
		}
		break;
	case SIVB_SIX_SWITCH_SELECTOR_FEEDBACK:
		if (ourSII)
		{
			int val = ourSII->GetSwitchSelector()->GetFeedback();
			return (val | ourSII->GetSIIToSIIInterstageConnector()->GetSwitchSelectorFeedback());
		}
		break;
	case SIVB_SIX_GETSITHRUSTOK:
		if (ourSII)
		{
			ourSII->GetSIIToSIIInterstageConnector()->GetSIThrustOK((bool*)m.val1.pValue, m.val2.iValue);
			return true;
		}
		break;
	case SIVB_SIX_GETSIITHRUSTOK:
		if (ourSII)
		{
			ourSII->GetThrustOK((bool*)m.val1.pValue);
			return true;
		}
		break;
	case SIVB_SIX_SI_PROPELLANT_DEPLETION_ENGINE_CUTOFF:
		if (ourSII)
		{
			m.val1.bValue = ourSII->GetSIIToSIIInterstageConnector()->GetSIPropellantDepletionEngineCutoff();
			return true;
		}
		break;
	case SIVB_SIX_SII_PROPELLANT_DEPLETION_ENGINE_CUTOFF:
		if (ourSII)
		{
			m.val1.bValue = ourSII->GetPropellantDepletionEngineCutoff();
			return true;
		}
		break;
	case SIVB_SIX_GET_SI_INBOARD_ENGINE_OUT:
		if (ourSII)
		{
			m.val1.bValue = ourSII->GetSIIToSIIInterstageConnector()->GetSIInboardEngineOut();
			return true;
		}
		break;
	case SIVB_SIX_GET_SI_OUTBOARD_ENGINE_OUT:
		if (ourSII)
		{
			m.val1.bValue = ourSII->GetSIIToSIIInterstageConnector()->GetSIOutboardEngineOut();
			return true;
		}
		break;
	case SIVB_SIX_GET_SII_FUEL_TANK_PRESSURE:
		if (ourSII)
		{
			m.val1.dValue = ourSII->GetLH2TankUllagePressurePSI();
			return true;
		}
		break;
	case SIVB_SIX_GET_SIX_SIVB_NOT_SEPARATED:
		if (ourSII)
		{
			m.val1.bValue = ourSII->GetSIISIVBNotSeparated();
			return true;
		}
		break;
	case SIVB_SIX_GET_SIC_SII_NOT_SEPARATED:
		if (ourSII)
		{
			m.val1.bValue = ourSII->GetSIIToSIIInterstageConnector()->GetSICSIINotSeparated();
			return true;
		}
		break;
	case SIVB_SIX_GET_SII_INTERSTAGE_NOT_SEPARATED:
		if (ourSII)
		{
			m.val1.bValue = ourSII->GetSIIToSIIInterstageConnector()->connectedTo != NULL; //TBD: Send to interstage, then back to S-II systems where a GSE simulate signal could be used
			return true;
		}
		break;
	}
	return false;
}

SIIToSIIInterstageConnector::SIIToSIIInterstageConnector()
{
	type = SII_SII_IS_COMMAND;
}

SIIToSIIInterstageConnector::~SIIToSIIInterstageConnector()
{

}

void SIIToSIIInterstageConnector::SetSIThrusterDir(int n, double yaw, double pitch)
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_SI_THRUSTER_DIR;
	cm.val1.iValue = n;
	cm.val2.dValue = yaw;
	cm.val3.dValue = pitch;

	if (SendMessage(cm))
	{
		return;
	}
}

void SIIToSIIInterstageConnector::SIEDSCutoff(bool cut)
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_SI_EDS_CUTOFF;
	cm.val1.bValue = cut;

	if (SendMessage(cm))
	{
		return;
	}
}

void SIIToSIIInterstageConnector::SISwitchSelector(int channel)
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_SI_SWITCH_SELECTOR;
	cm.val1.iValue = channel;

	SendMessage(cm);
}

int SIIToSIIInterstageConnector::GetSwitchSelectorFeedback()
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_SWITCH_SELECTOR_FEEDBACK;

	if (SendMessage(cm))
	{
		return cm.val1.iValue;
	}

	return 0;
}

void SIIToSIIInterstageConnector::GetSIThrustOK(bool* ok, int n)
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_GETSITHRUSTOK;
	cm.val1.pValue = ok;

	if (SendMessage(cm))
	{
		return;
	}

	for (int i = 0; i < n; i++)
	{
		ok[i] = false;
	}
}

bool SIIToSIIInterstageConnector::GetSIPropellantDepletionEngineCutoff()
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_PROPELLANT_DEPLETION_ENGINE_CUTOFF;

	if (SendMessage(cm))
	{
		return cm.val1.bValue;
	}

	return false;
}

bool SIIToSIIInterstageConnector::GetSIInboardEngineOut()
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_GET_SI_INBOARD_ENGINE_OUT;

	if (SendMessage(cm))
	{
		return cm.val1.bValue;
	}

	return false;
}

bool SIIToSIIInterstageConnector::GetSIOutboardEngineOut()
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_GET_SI_OUTBOARD_ENGINE_OUT;

	if (SendMessage(cm))
	{
		return cm.val1.bValue;
	}

	return false;
}

bool SIIToSIIInterstageConnector::GetSICSIINotSeparated()
{
	ConnectorMessage cm;

	cm.destination = SII_SII_IS_COMMAND;
	cm.messageType = SII_SIC_GET_SIC_SII_NOT_SEPARATED;

	if (SendMessage(cm))
	{
		return cm.val1.bValue;
	}

	return false;
}

SIISystems::SIISystems(Saturn* s, ThrustSpec* j2, TankSpec* j2prop, ThrustSpec* ull, int ull_num) : StageSystems(s),
	ourSwitchSelector(02, this),
	main_propellant(j2prop), ullage(ull),
	j2engine1(s, &j2[0]),
	j2engine2(s, &j2[1]),
	j2engine3(s, &j2[2]),
	j2engine4(s, &j2[3]),
	j2engine5(s, &j2[4]),
	ullage_num(ull_num)
{
	int i;

	for (i = 0; i < 5; i++)
	{
		ThrustOK[i] = false;
	}

	PUValveState = PUVALVE_NULL;

	PointLevelSensorArmed = false;
	UllageTrigger = false;
	PropellantDepletionSensors = false;
	OrdnanceArmed = false;
	SIISIVBOrdnanceArmed = false;
	StartPhaseLimiterCutoffArm = false;
	LH2StepPressurization = false;
	SICSIIEngineStartLockout = true;

	J2DefaultThrust = 0.0;

	LH2TankUllagePressurePSI = 50.0;

	j2engines[0] = &j2engine1;
	j2engines[1] = &j2engine2;
	j2engines[2] = &j2engine3;
	j2engines[3] = &j2engine4;
	j2engines[4] = &j2engine5;

	siiSIIInterstageConnector.SetSIISystems(this);
	siiSIVBConnector.SetSIISystems(this);
}

SIISystems::~SIISystems()
{
	siiSIIInterstageConnector.Disconnect();
	siiSIVBConnector.Disconnect();
}

void SIISystems::Timestep(double simdt)
{
	if (j2engines[0] == NULL) return;

	// S-IC/S-II Engine Start Lockout
	SICSIIEngineStartLockout = siiSIIInterstageConnector.GetSICSIINotSeparated();

	j2engine1.Timestep(simdt);
	j2engine2.Timestep(simdt);
	j2engine3.Timestep(simdt);
	j2engine4.Timestep(simdt);
	j2engine5.Timestep(simdt);

	//Propellant systems
	if (main_propellant)
	{
		LH2TankUllagePressurePSI = 0.0; // TBD: vessel->GetPropellantMass(main_propellant) / vessel->GetPropellantMaxMass(main_propellant) * 50.0;
	}

	//Thrust OK switch
	for (int i = 0; i < 5; i++)
	{
		ThrustOK[i] = j2engines[i]->GetThrustOK();
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

	//Ullage engines
	if (OrdnanceArmed && ullage && UllageTrigger)
	{
		for (int i = 0; i < ullage_num; i++)
		{
			GetSat()->SetThrusterLevel(&ullage[i], 1.0);
		}
	}
}

void SIISystems::SetEngineFailed(int n, bool fail)
{
	if (n < 0 || n > 4) return;

	j2engines[n]->SetFailed(fail);
}

#define MR_STATS 5

static double MixtureRatios[MR_STATS] = { 6.0, 5.5, 5.0, 4.3, 4.0 };
//static double MRISP[MR_STATS] = { 416*G, 418*G, 421*G, 427*G, 432*G };
static double MRISP[MR_STATS] = { 421.4 * G, 423.4 * G, 426.5 * G, 432.6 * G, 437.6 * G };
static double MRThrust[MR_STATS] = { 1.1, 1.0, .898, .7391, .7 };

void SIISystems::GetJ2ISP(double ratio, double& isp, double& ThrustAdjust)

{
	isp = 421 * G;

	// From Usenet:
	// It had roughly three stops. 178,000 lbs at 425s Isp and an O/F of 4.5,
	// 207,000 lbs at 421s Isp and an O/F of 5.0, and 230,500 lbs at 418s Isp
	// and an O/F of 5.5.

	for (int i = 0; i < MR_STATS; i++) {
		if (ratio >= MixtureRatios[i]) {
			double delta = (ratio - MixtureRatios[i]) / (MixtureRatios[i - 1] - MixtureRatios[i]);

			isp = MRISP[i] + ((MRISP[i - 1] - MRISP[i]) * delta);
			ThrustAdjust = MRThrust[i] + ((MRThrust[i - 1] - MRThrust[i]) * delta);

			return;
		}
	}
}

void SIISystems::SetPUValve(int state)
{
	if (state == PUVALVE_CLOSED)
	{
		PUValveState = PUVALVE_CLOSED;
	}
	else if (state == PUVALVE_NULL)
	{
		PUValveState = PUVALVE_NULL;
	}
	else if (state == PUVALVE_OPEN)
	{
		PUValveState = PUVALVE_OPEN;
	}

	RecalculateEngineParameters();
}

void SIISystems::RecalculateEngineParameters(double BaseThrust)
{
	J2DefaultThrust = BaseThrust;

	RecalculateEngineParameters();
}

void SIISystems::RecalculateEngineParameters()
{
	double MixtureRatio;

	if (PUValveState == PUVALVE_CLOSED)
	{
		MixtureRatio = 5.5;
	}
	else if (PUValveState == PUVALVE_OPEN)
	{
		MixtureRatio = 4.5;
	}
	else
	{
		MixtureRatio = 5.0;
	}

	SetMixtureRatio(MixtureRatio);
}

void SIISystems::SetMixtureRatio(double ratio)
{
	double isp, thrust, ThrustAdjust;

	GetJ2ISP(ratio, isp, ThrustAdjust);
	thrust = J2DefaultThrust * ThrustAdjust;

	for (int i = 0; i < 5; i++) {
		j2engines[i]->SetThrusterIsp(isp);
		j2engines[i]->SetThrusterMax0(thrust);
	}
}

void SIISystems::SetThrusterDir(int n, double beta_y, double beta_p)
{
	if (n < 0 || n > 3) return;

	j2engines[n]->SetThrusterDir(beta_y, beta_p);
}

bool SIISystems::PropellantLowLevel()
{
	if (PointLevelSensorArmed)
	{
		if (main_propellant)
		{
			//Actual propellant remaining for Apollo 11 was 7368.0 kg (including unusable)
			if (GetSat()->GetPropellantMass(main_propellant) < 7368.0) //3866.0
			{
				return true;
			}
		}
	}

	return false;
}

void SIISystems::GetThrustOK(bool* ok)
{
	for (int i = 0; i < 5; i++)
	{
		ok[i] = ThrustOK[i];
	}
}

bool SIISystems::GetPropellantDepletionEngineCutoff()
{
	if (PointLevelSensorArmed)
	{
		if (PropellantDepletionSensors) return true;

		for (int i = 0; i < 5; i++) if (ThrustOK[i]) return false;

		return true;
	}

	return false;
}

bool SIISystems::GetSIISIVBNotSeparated()
{
	//TBD: ESE S-II/S-IVB sep simulate
	return true;
}

void SIISystems::SwitchSelectorOutput(int channel)
{
	GetSat()->SendEventMessage("S-II Switch Selector Channel " + std::to_string(channel));

	switch (channel)
	{
	case 3: //S-II LOX Depletion Sensors Cutoff Arm
		SetDepletionSensorsCutoffArm();
		break;
	case 5: //S-II/S-IVB Separation
		if (SIISIVBOrdnanceArmed)
		{
			SII_SIVB_Separation_Pyros.SetBlown(true);
		}
		break;
	case 6: //Start Phase Limiter Cutoff Reset
		StartPhaseLimiterCutoffArmReset();
		break;
	case 7: //LH2 Step Pressurization
		LH2StepPressurization = true;
		break;
	case 8: //S-II/S-IVB Ordnance Arm
		SetSIISIVBOrdnanceArm();
		break;
	case 9: //Stop First PAM - FM/FM Calibration
		break;
	case 11: //S-II Ordnance Arm
		SetOrdnanceArm();
		break;
	case 12: //Hydraulic Accumulators Unlock
		break;
	case 14: //LOX Step Pressurization
		break;
	case 15: //S-II Center Engine Cutoff
		LVDCCenterEngineCutoff();
		break;
	case 18: //S-II Engines Cutoff
		LVDCEnginesCutoff();
		break;
	case 19: //Prevalves Lockout Reset
		break;
	case 20: //Engines Ready Bypass
		SetEnginesReadyBypass();
		break;
	case 23: //S-II Aft Interstage Separation
		if (OrdnanceArmed)
		{
			SII_Interstage_Pyros.SetBlown(true);
		}
		break;
	case 24: //S-II Ullage Trigger
		FireUllageTrigger();
		break;
	case 25: //Start Phase Limiter Cutoff Arm
		StartPhaseLimiterCutoffArm = true;
		break;
	case 30: //Start First PAM - FM/FM Relays Reset
		break;
	case 31: //S-II Engines Cutoff Reset
		LVDCEnginesCutoffReset();
		break;
	case 32: //PU System Activate
		break;
	case 33: //S-II Engine Start
		EngineStartOn();
		break;
	case 38: //LH2 Tank High Pressure Vent Mode
		break;
	case 42: //S-II LH2 Depletion Sensors Cutoff Arm
		SetDepletionSensorsCutoffArm();
		break;
	case 48: //LH2 Recirculation Pumps Off
		break;
	case 49: //Engines Ready Bypass Reset
		ResetEnginesReadyBypass();
		break;
	case 56: //Low (4.5) Engine MR Ratio On
		SetPUValve(PUVALVE_OPEN);
		//puShiftSound.play(NOLOOP);
		//puShiftSound.done();
		break;
	case 58: //High (5.5) Engine MR Ratio Off
		SetPUValve(PUVALVE_NULL);
		break;
	case 59: //High (5.5 Engine Mixture Ratio On
		SetPUValve(PUVALVE_CLOSED);
		break;
	case 60: //PU System Open Loop Arm
		break;
	case 66: //Start Recorder Times
		break;
	case 71: //Start Data Recorders
		break;
	case 88: //Chilldown Valves Close
		break;
	case 90: //Measurement Control Switch No. 2 Activate
		break;
	case 99: //Prevalves Close Arm
		break;
	case 104: //Stop Data Recorders
		break;
	default:
		break;
	}
}

void SIISystems::LVDCEnginesCutoff()
{
	j2engine1.SetLVDCEngineCutoff();
	j2engine2.SetLVDCEngineCutoff();
	j2engine3.SetLVDCEngineCutoff();
	j2engine4.SetLVDCEngineCutoff();
	j2engine5.SetLVDCEngineCutoff();
}

void SIISystems::LVDCEnginesCutoffReset()
{
	j2engine1.ResetLVDCEnginesCutoff();
	j2engine2.ResetLVDCEnginesCutoff();
	j2engine3.ResetLVDCEnginesCutoff();
	j2engine4.ResetLVDCEnginesCutoff();
	j2engine5.ResetLVDCEnginesCutoff();
}

void SIISystems::LVDCCenterEngineCutoff()
{
	j2engine5.SetLVDCEngineCutoff();
}

void SIISystems::SetEnginesReadyBypass()
{
	j2engine1.SetEngineReadyBypass();
	j2engine2.SetEngineReadyBypass();
	j2engine3.SetEngineReadyBypass();
	j2engine4.SetEngineReadyBypass();
	j2engine5.SetEngineReadyBypass();
}

void SIISystems::ResetEnginesReadyBypass()
{
	j2engine1.ResetEngineReadyBypass();
	j2engine2.ResetEngineReadyBypass();
	j2engine3.ResetEngineReadyBypass();
	j2engine4.ResetEngineReadyBypass();
	j2engine5.ResetEngineReadyBypass();
}

void SIISystems::EngineStartOn()
{
	j2engine1.SetEngineStart();
	j2engine2.SetEngineStart();
	j2engine3.SetEngineStart();
	j2engine4.SetEngineStart();
	j2engine5.SetEngineStart();
}

void SIISystems::EngineStartOff()
{
	j2engine1.ResetEngineStart();
	j2engine2.ResetEngineStart();
	j2engine3.ResetEngineStart();
	j2engine4.ResetEngineStart();
	j2engine5.ResetEngineStart();
}

void SIISystems::EDSEnginesCutoff(bool cut)
{
	if (cut)
	{
		j2engine1.SetEDSCutoff();
		j2engine2.SetEDSCutoff();
		j2engine3.SetEDSCutoff();
		j2engine4.SetEDSCutoff();
		j2engine5.SetEDSCutoff();
	}
	else
	{
		j2engine1.ResetEDSCutoff();
		j2engine2.ResetEDSCutoff();
		j2engine3.ResetEDSCutoff();
		j2engine4.ResetEDSCutoff();
		j2engine5.ResetEDSCutoff();
	}
}

SIIInterstageSystemsConnector::SIIInterstageSystemsConnector()
{
	ourIS = NULL;
}

SIIInterstageSystemsConnector::~SIIInterstageSystemsConnector()
{

}

SIIInterstageToSIIConnector::SIIInterstageToSIIConnector()
{
	type = SII_SII_IS_COMMAND;
}

SIIInterstageToSIIConnector::~SIIInterstageToSIIConnector()
{

}

bool SIIInterstageToSIIConnector::ReceiveMessage(Connector* from, ConnectorMessage& m)
{
	//
	// Sanity check.
	//

	if (m.destination != type)
	{
		return false;
	}

	//Directly forward the message
	return ourIS->GetSIIInterstageToSICConnector()->ForwardMessage(m);
}

SIIInterstageToSICConnector::SIIInterstageToSICConnector()
{
	type = SII_IS_SIC_COMMAND;
}

SIIInterstageToSICConnector::~SIIInterstageToSICConnector()
{

}

bool SIIInterstageToSICConnector::ForwardMessage(ConnectorMessage& m)
{
	//Directly forward the message
	m.destination = type;

	return SendMessage(m);
}

SIIInterstageSystems::SIIInterstageSystems()
{
	siiInterstageSIIConnector.SetSIIInterstageSystems(this);
	siiInterstageSICConnector.SetSIIInterstageSystems(this);
}

SIIInterstageSystems::~SIIInterstageSystems()
{
	siiInterstageSIIConnector.Disconnect();
	siiInterstageSICConnector.Disconnect();
}