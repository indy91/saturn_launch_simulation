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

#include "TailUmbilical.h"
#include "SIC_ESE.h"

SIC_ESE::SIC_ESE() : SI_ESE()
{
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			SICThrustOKSimulate[i][j] = false;
			SICThrustOK[i * 3 + j] = false;
		}
	}
}

void SIC_ESE::Timestep()
{
	siESEToSICommandConnector.GetSIThrustOK(SICThrustOK, 15);
}