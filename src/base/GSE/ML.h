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

#include "RCA110A.h"

class SI_ESE;
class Connector;
class IU_ESE;
class Saturn;

// Mobile Launcher

class ML
{
public:
	ML(Saturn* s);
	~ML();

	void Init(double gmtlo, bool IsSaturnV);

	void PreStep(double t, double simdt);
	void PostStep(double t, double simdt);

	void ConnectSIUmbilical(Connector *c);
	void ConnectIUUmbilical(Connector* c);
	void DisconnectUmbilicals();

	RCA110A* GetRCA() { return &rca110A; }
private:
	Saturn* GetSat();

	bool CutoffInterlock();
	bool Commit();

	int EventCounter;

	SI_ESE* SIESE;
	IU_ESE* IUESE;
	RCA110A rca110A;
	Saturn* sat;
};