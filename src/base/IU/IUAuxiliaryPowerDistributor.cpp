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

#include "iu.h"
#include "IUAuxiliaryPowerDistributor.h"

IUAuxiliaryPowerDistributor1::IUAuxiliaryPowerDistributor1(IU* iu)
{
	this->iu = iu;

	MotorSwitchLogic = false;
	CommandPowerTransfer = false;
}

void IUAuxiliaryPowerDistributor1::Timestep(double simdt)
{
	if (CommandPowerTransfer && iu->GetIUToIUESECommandConnector()->IsUmbilicalConnected())
		MotorSwitchLogic = true;
	else
		MotorSwitchLogic = false;
}

IUAuxiliaryPowerDistributor2::IUAuxiliaryPowerDistributor2(IU* iu)
{
	this->iu = iu;

	EDSBus1PowerOff = false;
	EDSBus2PowerOff = false;
	EDSBus3PowerOff = false;
	ControlSignalProcessorPowerOn = true;
}

void IUAuxiliaryPowerDistributor2::Timestep(double simdt)
{
	if (iu->GetIUToIUESECommandConnector()->GetEDSPowerInhibit())
	{
		EDSBus1PowerOff = true;
		EDSBus2PowerOff = true;
		EDSBus3PowerOff = true;
	}
	else
	{
		EDSBus1PowerOff = false;
		EDSBus2PowerOff = false;
		EDSBus3PowerOff = false;
	}
}