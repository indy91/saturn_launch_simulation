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

class IU;

// 601A33
class IUAuxiliaryPowerDistributor1
{
public:
	IUAuxiliaryPowerDistributor1(IU* iu);
	void Timestep(double simdt);

	void CommandPowerTransferInternal() { CommandPowerTransfer = true; }
	void CommandPowerTransferExternal() { CommandPowerTransfer = false; }
protected:
	//K1
	bool MotorSwitchLogic;
	//K8
	bool CommandPowerTransfer;

	IU* iu;
};

// 602A34
class IUAuxiliaryPowerDistributor2
{
public:
	IUAuxiliaryPowerDistributor2(IU* iu);
	void Timestep(double simdt);

	bool IsIUEDSBusPowered() { return (EDSBus1PowerOff == false); }
	bool IsControlSignalProcessorPowered() { return ControlSignalProcessorPowerOn; }

	void SetControlSignalProcessorPower(bool set) { ControlSignalProcessorPowerOn = set; }
protected:

	// K18 (K116)
	bool EDSBus1PowerOff;
	// K15 (K117)
	bool EDSBus2PowerOff;
	// K13 (K118)
	bool EDSBus3PowerOff;
	// K17-1/2 (K247-1/2)
	bool ControlSignalProcessorPowerOn;

	IU* iu;
};