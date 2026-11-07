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

#include "../util/VectorMath.h"

///
/// \ingroup Connectors
/// \brief Connector type.
///
enum ConnectorType
{
	NO_CONNECTION,				///< Dummy.
	SIVB_SIX_COMMAND,			///< Docking connector between S-IVB and S-IB/S-II
	SII_SII_IS_COMMAND,			///< Docking connector between S-II and S-II Interstage
	SII_IS_SIC_COMMAND,			///< Docking connector between S-II Interstage and S-IC
	SIVB_IU_COMMAND,			///< Passes commands between S-IVB and IU.
	IUESE_IU_COMMAND,			///< Passes commands between IU ESE and IU
	SIESE_SI_COMMAND,			///< Passes commands between SI ESE and S-I
};

//
// The message can pass various different parameters in this union. The receiver
// will determine which is correct based on the message type.
//
// \brief Message value for connector messages.
//
union ConnectorMessageValue
{
	int iValue;				///< Integer message value.
	double dValue;			///< Floating point message value.
	bool bValue;			///< Boolean message value.
	void* pValue;			///< Pointer message value.
	VECTOR3 vValue;			///< Vector message value;
};

///
/// \ingroup Connectors
/// \brief Connector message to be passed through the system.
///
struct ConnectorMessage
{
	///
	/// \brief What kind of connector should this message go to?
	///
	ConnectorType destination;

	///
	/// \brief Connection-specific message type.
	///
	unsigned int messageType;

	///
	/// \brief Message value 1.
	///
	ConnectorMessageValue val1;

	///
	/// \brief Message value 2.
	///
	ConnectorMessageValue val2;

	///
	/// \brief Message value 3.
	///
	ConnectorMessageValue val3;

	///
	/// \brief Message value 4.
	///
	ConnectorMessageValue val4;
};

///
/// \ingroup Connectors
/// \brief Connector class. Specific connectors will be derived from this class.
///
class Connector {
public:
	///
	/// \brief Constructor.
	///
	Connector();

	///
	/// \brief Destructor. Disconnects the connector when called.
	///
	virtual ~Connector();

	///
	/// \brief Get the type of the connector.
	/// \return Connector type.
	///
	virtual ConnectorType GetType();

	///
	/// \brief Set the type of the connector.
	/// \param t Connector type.
	///
	void SetType(ConnectorType t) { type = t; };

	///
	/// \brief Connect to another connector.
	/// \param other Other end of the connection.
	/// \return True if the connector is the correct type and we connected.
	///
	virtual bool ConnectTo(Connector* other);

	///
	/// \brief Disconnect from the far end of the connection.
	///
	virtual void Disconnect();

	///
	/// \brief Has been disconnected from the far end of the connection.
	///
	virtual void Disconnected();

	///
	/// Send a message through the connector. Note that the receiver can update the value in the
	/// connector message, in order to return data to the caller.
	///
	/// \brief Send message.
	/// \param m Message to send.
	/// \return False if the connector isn't connected to anything or the message wasn't handled
	/// at the far end of the connection.
	///
	virtual bool SendMessage(ConnectorMessage& m);

	///
	/// Receieve a message through the connector. Note that the receiver can update the value in the
	/// connector message, in order to return data to the caller.
	///
	/// The default connector is output-only, and will return an error if the other end of the connection
	/// tries to send it data.
	///
	/// \brief Receive message.
	/// \param from The connector that sent the message.
	/// \param m Message received.
	/// \return False if we don't handle the message.
	///
	virtual bool ReceiveMessage(Connector* from, ConnectorMessage& m);

	///
	/// \brief Connector we're connected to, if any.
	///
	Connector* connectedTo;

protected:
	///
	/// \brief Type of connection.
	///
	ConnectorType type;
};