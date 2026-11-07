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

class LCCPadInterface;

#include "TailUmbilical.h"

class SI_ESE
{
public:
	SI_ESE();
	virtual ~SI_ESE();

	//S-I ESE to S-I
	virtual bool GetSIThrustOKSimulate(int eng, int n) = 0;
	//S-I ESE to ML
	virtual bool GetSIThrustOK(int eng, int n) = 0;

	virtual void Timestep() = 0;

	SIESEToSICommandConnector* GetSIESEToSICommandConnector() { return &siESEToSICommandConnector; }
protected:
	SIESEToSICommandConnector siESEToSICommandConnector;
};