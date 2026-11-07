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

#include "SI_ESE.h"

class SIC_ESE : public SI_ESE
{
public:
	SIC_ESE();

	void Timestep();

	bool GetSIThrustOKSimulate(int eng, int n) { return SICThrustOKSimulate[eng - 1][n - 1]; }
	bool GetSIThrustOK(int eng, int n) { return SICThrustOK[(eng - 1) * 3 + (n - 1)]; }
protected:
	bool SICThrustOKSimulate[5][3];
	bool SICThrustOK[15];
};