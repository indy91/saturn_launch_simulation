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

#include "connector.h"

Connector::Connector()

{
	type = NO_CONNECTION;
	connectedTo = 0;
}

Connector::~Connector()

{
	//
	// We have to be sure to disconnect before deletion so the other end doesn't try to send
	// us any more data.
	//
	Disconnect();
}

void Connector::Disconnect()

{
	if (connectedTo)
	{
		connectedTo->connectedTo = 0;
		connectedTo = 0;
	}
}

void Connector::Disconnected()

{
	//
	// Do nothing by default.
	//
}

bool Connector::ConnectTo(Connector* other)

{
	//
	// Other may be NULL. If so just clear this connection, otherwise
	// check the other end is the correct type and disconnected.
	//
	if (other)
	{
		if (other->GetType() != type || other->connectedTo)
		{
			return false;
		}

		other->connectedTo = this;
	}

	connectedTo = other;

	return true;
}

bool Connector::SendMessage(ConnectorMessage& m)

{
	if (connectedTo)
	{
		return connectedTo->ReceiveMessage(this, m);
	}

	return false;
}

bool Connector::ReceiveMessage(Connector* from, ConnectorMessage& m)

{
	//
	// The default connector is one-way. Return an error if anything tries to send data to us.
	//
	return false;
}

ConnectorType Connector::GetType()

{
	return type;
}