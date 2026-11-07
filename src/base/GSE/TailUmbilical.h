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

#include "../connector.h"

class SI_ESE;

//IU ESE to IU connector
class SIESEToSICommandConnector : public Connector
{
public:
	SIESEToSICommandConnector();
	~SIESEToSICommandConnector();

	//S-I to S-I ESE
	bool ReceiveMessage(Connector* from, ConnectorMessage& m);

	//S-I ESE to S-I
	bool SIStageLogicCutoff();
	void SetEngineStart(int eng);
	void SIGSECutoff(bool cut);
	void GetSIThrustOK(bool* ok, int n);

	void SetSI_ESE(SI_ESE* si_ese) { ourSI_ESE = si_ese; };
protected:
	SI_ESE* ourSI_ESE;
};