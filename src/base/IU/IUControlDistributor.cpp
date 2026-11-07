#include "iu.h"
#include "IUControlDistributor.h"
#include "../../util/Definitions.h"

IUControlDistributor::IUControlDistributor(IU* iu)
{
	this->iu = iu;

	FCCPowerOn = true;
	ControlSignalProcessorPowerOn = true;
	IsSIVBBurnModeA = false;
	IsSIVBBurnModeB = false;
	for (int i = 0; i < 5; i++)
	{
		SwitchPoint1to5[i] = false;
	}
	SIVBThrustNotOK = false;
	GSECommandVehicleLiftoffIndicationInhibit = true; // TBD
	ExcessiveRatePYRAutoAbortInhibitEnable = false;
	TwoEngOutAutoAbortInhibit = false;
	ExcessiveRatePYRAutoAbortInhibit = false;
	ExcessiveRateRollAutoAbortInhibit = false;
	ExcessiveRateRollAutoAbortInhibitEnable = false;
	TwoEngOutAutoAbortInhibitEnable = false;
	IUCommandSystemEnable = false;
	QBallPowerOn = true;
	SIVBEngineOutA = false;
	SIVBEngineOutB = false;
	GuidanceReferenceRelease = false;
}

void IUControlDistributor::Timestep(double simdt)
{
	if (iu->GetIUToIUESECommandConnector()->GetCommandVehicleLiftoffIndicationInhibit())
		GSECommandVehicleLiftoffIndicationInhibit = true;
	else
		GSECommandVehicleLiftoffIndicationInhibit = false;

	GuidanceReferenceRelease = iu->GetIUToIUESECommandConnector()->GetGuidanceReferenceRelease();

	if (iu->GetSIVBCommandConnector()->GetSIVBThrustOK() == false)
	{
		SIVBThrustNotOK = true;
		SIVBEngineOutA = true;
		SIVBEngineOutB = true;
	}
	else
	{
		SIVBThrustNotOK = false;
		SIVBEngineOutA = false;
		SIVBEngineOutB = false;
	}

	if (iu->GetEDS()->GetIUCommandSystemEnable())
		IUCommandSystemEnable = true;
	else
		IUCommandSystemEnable = false;
}

bool IUControlDistributor::GetSIVBBurnMode()
{
	if (IsSIVBBurnModeA && !SIVBThrustNotOK) return true;
	if (IsSIVBBurnModeA && SIVBThrustNotOK && IsSIVBBurnModeB) return true;
	if (IsSIVBBurnModeB && !SIVBThrustNotOK) return true;

	return false;
}

VECTOR3 IUControlDistributor::GetAttitudeError()
{
	return iu->GetLVDA()->GetAttitudeError();
}

bool IUControlDistributor::GetTwoEnginesOutAutoAbortInhibit()
{
	return (TwoEngOutAutoAbortInhibitEnable && TwoEngOutAutoAbortInhibit);
}

bool IUControlDistributor::GetExcessiveRatePYRAutoAbortInhibit()
{
	return (ExcessiveRatePYRAutoAbortInhibitEnable && ExcessiveRatePYRAutoAbortInhibit);
}

bool IUControlDistributor::GetExcessiveRateRollAutoAbortInhibit()
{
	return (ExcessiveRateRollAutoAbortInhibitEnable && ExcessiveRateRollAutoAbortInhibit);
}

void IUControlDistributor::SwitchSelector(int cmd)
{
	// Send to all
	iu->GetSIVBCommandConnector()->SISwitchSelector(cmd);
	iu->GetSIVBCommandConnector()->SIISwitchSelector(cmd);
	iu->GetSIVBCommandConnector()->SIVBSwitchSelector(cmd);
	iu->GetSwitchSelector()->ProcessCommand(cmd);
}

int IUControlDistributor::GetSwitchSelectorFeedback()
{
	int val = iu->GetSwitchSelector()->GetFeedback();

	return (val | iu->GetSIVBCommandConnector()->GetSwitchSelectorFeedback());
}

void IUControlDistributor::ResetBus1()
{
	IsSIVBBurnModeB = false;
	ExcessiveRatePYRAutoAbortInhibitEnable = false;
	ExcessiveRateRollAutoAbortInhibitEnable = false;
	TwoEngOutAutoAbortInhibit = false;

	iu->GetEDS()->ResetBus1();
}

void IUControlDistributor::ResetBus2()
{
	IsSIVBBurnModeA = false;
	ExcessiveRatePYRAutoAbortInhibit = false;
	ExcessiveRateRollAutoAbortInhibit = false;
	TwoEngOutAutoAbortInhibitEnable = false;
	SwitchPoint1to5[0] = false;
	SwitchPoint1to5[1] = false;
	SwitchPoint1to5[2] = false;
	SwitchPoint1to5[3] = false;
	SwitchPoint1to5[4] = false;

	iu->GetEDS()->ResetBus2();
}

bool IUControlDistributor::GetEDSEnableLogicZero()
{
	return !(ExcessiveRatePYRAutoAbortInhibit || ExcessiveRatePYRAutoAbortInhibitEnable || ExcessiveRateRollAutoAbortInhibit || ExcessiveRateRollAutoAbortInhibitEnable || TwoEngOutAutoAbortInhibit || TwoEngOutAutoAbortInhibitEnable);
}

IUControlDistributor1B::IUControlDistributor1B(IU* iu) : IUControlDistributor(iu)
{

}

void IUControlDistributor1B::Timestep(double simdt)
{
	IUControlDistributor::Timestep(simdt);
}

bool IUControlDistributor1B::GetSIBurnMode()
{
	//GSE S-IC Burn Mode Substitute
	if (iu->GetIUToIUESECommandConnector()->GetSIBurnModeSubstitute()) return true;
	//Normal S-I Burn Mode Logic
	if (iu->GetSIVBCommandConnector()->SIXSIVBNotSeparated() && !GSECommandVehicleLiftoffIndicationInhibit) return true;

	return false;
}

IUControlDistributorSV::IUControlDistributorSV(IU* iu) : IUControlDistributor(iu)
{
	IsSIIBurnMode = false;
	SICOutboardEnginesCantInhibit = false;
	SICEngineCantA = false;
	SICEngineCantB = false;
	SICEngineCantC = false;
	for (int i = 0; i < 4; i++)
	{
		SwitchPoint6to9[i] = false;
	}
}

void IUControlDistributorSV::Timestep(double simdt)
{
	IUControlDistributor::Timestep(simdt);

	if (iu->GetIUToIUESECommandConnector()->GetSICOutboardEnginesCantInhibit())
		SICOutboardEnginesCantInhibit = true;
	else
		SICOutboardEnginesCantInhibit = false;

	//sprintf(oapiDebugString(), "%d %d %d %d %d %d", SwitchPoint1, SwitchPoint2, SwitchPoint3, SwitchPoint4, SwitchPoint5, SwitchPoint6);
}

bool IUControlDistributorSV::GetSIBurnMode()
{
	//GSE S-IC Burn Mode Substitute
	if (iu->GetIUToIUESECommandConnector()->GetSIBurnModeSubstitute()) return true;
	//Normal S-IC Burn Mode Logic
	if (iu->GetSIVBCommandConnector()->SIXSIVBNotSeparated() && !IsSIIBurnMode && !GSECommandVehicleLiftoffIndicationInhibit) return true;

	return false;
}

bool IUControlDistributorSV::GetSIIBurnMode()
{
	if (iu->GetSIVBCommandConnector()->SIXSIVBNotSeparated() && IsSIIBurnMode && !GSECommandVehicleLiftoffIndicationInhibit)
		return true;

	return false;
}

bool IUControlDistributorSV::UseSICEngineCant()
{
	if (iu->GetSIVBCommandConnector()->SICSIINotSeparated() && SICEngineCantC && !SICOutboardEnginesCantInhibit)
	{
		if (SICEngineCantA) return true;
		else if (SICEngineCantB) return true;
	}
	else if (SICEngineCantA && SICEngineCantB && !SICOutboardEnginesCantInhibit) return true;
	else if (iu->GetIUToIUESECommandConnector()->GetSICOutboardEnginesCantSimulate()) return true;

	return false;
}

void IUControlDistributorSV::SetSIIBurnModeEngineCantOff()
{
	IsSIIBurnMode = true;
	SICEngineCantA = false;
}

void IUControlDistributorSV::ResetBus1()
{
	IUControlDistributor::ResetBus1();

	IsSIIBurnMode = false;
	if (!GSECommandVehicleLiftoffIndicationInhibit)
	{
		SICEngineCantB = false;
	}
	SwitchPoint6to9[3] = false;
}

void IUControlDistributorSV::ResetBus2()
{
	IUControlDistributor::ResetBus2();

	SICEngineCantC = false;
	if (!GSECommandVehicleLiftoffIndicationInhibit)
	{
		SICEngineCantA = false;
	}
	SwitchPoint6to9[0] = false;
	SwitchPoint6to9[1] = false;
	SwitchPoint6to9[2] = false;
}