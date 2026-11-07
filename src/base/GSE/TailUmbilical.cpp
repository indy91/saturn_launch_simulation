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
#include "SI_ESE.h"
#include "../connector_messages.h"

SIESEToSICommandConnector::SIESEToSICommandConnector()
{
	type = SIESE_SI_COMMAND;
	ourSI_ESE = 0;
}

SIESEToSICommandConnector::~SIESEToSICommandConnector()
{

}

bool SIESEToSICommandConnector::ReceiveMessage(Connector* from, ConnectorMessage& m)
{
	//
	// Sanity check.
	//

	if (m.destination != type)
	{
		return false;
	}

	SIESEMessageType messageType;

	messageType = (SIESEMessageType)m.messageType;

	switch (messageType)
	{
	case SI_SIESE_GET_SI_THRUST_OK_SIMULATE:
		if (ourSI_ESE)
		{
			m.val1.bValue = ourSI_ESE->GetSIThrustOKSimulate(m.val1.iValue, m.val2.iValue);
			return true;
		}
		break;
	}

	return false;
}

bool SIESEToSICommandConnector::SIStageLogicCutoff()
{
	ConnectorMessage m;

	m.destination = SIESE_SI_COMMAND;
	m.messageType = SIESE_SI_SI_STAGE_LOGIC_CUTOFF;

	if (SendMessage(m))
	{
		return m.val1.bValue;
	}

	return false;
}

void SIESEToSICommandConnector::SetEngineStart(int eng)
{
	ConnectorMessage m;

	m.destination = SIESE_SI_COMMAND;
	m.messageType = SIESE_SI_SET_ENGINE_START;
	m.val1.iValue = eng;

	SendMessage(m);
}

void SIESEToSICommandConnector::SIGSECutoff(bool cut)
{
	ConnectorMessage m;

	m.destination = SIESE_SI_COMMAND;
	m.messageType = SIESE_SI_GSE_CUTOFF;
	m.val1.bValue = cut;

	SendMessage(m);
}

void SIESEToSICommandConnector::GetSIThrustOK(bool* ok, int n)
{
	ConnectorMessage m;

	m.destination = SIESE_SI_COMMAND;
	m.messageType = SIESE_SI_THRUST_OK;
	m.val1.pValue = ok;

	if (SendMessage(m))
	{
		return;
	}

	for (int i = 0; i < n; i++)
	{
		ok[i] = false;
	}
}