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

class StageSystems;

class SwitchSelector
{
public:
	SwitchSelector(int stag, StageSystems* sys);

	void ProcessCommand(int cmd);
	int GetFeedback();
protected:

	int CodeToChannel(int code) const;

	// Relays K1-K8
	int InputRelays;
	// Relays K9-16
	int FeedbackRelays;
	// Relays K20-K22
	bool StageSelect;

	// 
	const int stage;

	StageSystems* StageSys;
};