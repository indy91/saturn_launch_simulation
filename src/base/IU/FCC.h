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

class FCC
{
public:
	FCC(IU* iu);
	virtual ~FCC() {}
	virtual void Timestep(double simdt) = 0;
protected:
	//K1-1/2
	bool SIBurnMode;
	//K2
	bool SIVBBurnMode;
	//K35/K37
	bool SCControlEnableRelay;

	double a_0p, a_0y, a_0r;
	double a_1p, a_1y, a_1r;
	double beta_pc, beta_yc, beta_rc;
	double beta_y1c, beta_y2c, beta_y3c, beta_y4c;
	double beta_p1c, beta_p2c, beta_p3c, beta_p4c;
	double eps_p, eps_ymr, eps_ypr;

	IU* iu;
};

class FCC1B : public FCC
{
public:
	FCC1B(IU* iu);
	void Timestep(double simdt);
};

class FCCSV : public FCC
{
public:
	FCCSV(IU* iu);
	void Timestep(double simdt);
protected:
	//K2-1/2
	bool SIIBurnMode;
	//K4
	bool SICOrSIIBurnMode;
	bool UseSICEngineCant;
	double SICCant;
};