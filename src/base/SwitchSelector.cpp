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

#include "SwitchSelector.h"
#include "StageSystems.h"
#include "../util/Definitions.h"

static const int Code[] = { 36, 5, 30, 74, 81, 97, 63, 0,
87, 111, 94, 49, 14, 51, 39, 0,
16, 24, 9, 75, 109, 82, 71, 0,
68, 57, 96, 33, 7, 35, 17, 0,
53, 23, 31, 61, 93, 102, 90, 0,
69, 65, 73, 48, 6, 52, 40, 0,
38, 44, 18, 110, 92, 101, 67, 0,
58, 64, 95, 29, 25, 36, 8, 0,
21, 46, 32, 98, 108, 76, 89, 0,
66, 62, 112, 105, 12, 41, 34, 0,
22, 19, 45, 56, 91, 99, 86, 0,
84, 100, 60, 88, 27, 42, 2, 0,
4, 3, 10, 55, 107, 77, 85, 0,
83, 79, 59, 70, 47, 20, 15, 0,
50, 13, 11, 54, 106, 78, 104, 0,
103, 80, 72, 28, 26, 43, 1 };

SwitchSelector::SwitchSelector(int stag, StageSystems* sys) : stage(stag)
{
	StageSys = sys;

	InputRelays = 0;
	StageSelect = false;
	FeedbackRelays = false;
}

void SwitchSelector::ProcessCommand(int cmd)
{
	SwitchSelectorCommand cmd2;

	cmd2.Value = cmd;

	// Is stage selected?
	if (cmd2.Bits.Reset)
	{
		StageSelect = false;
	}
	else if ((cmd2.Bits.StageSelect & stage) != 0)
	{
		StageSelect = true;
	}

	// Input relays
	if (cmd2.Bits.Reset)
	{
		InputRelays = 0;
	}
	else if (StageSelect)
	{
		InputRelays = cmd2.Bits.FlightCommand;
	}

	// Read command
	if (StageSelect && cmd2.Bits.Read)
	{
		// Issue command
		StageSys->SwitchSelectorOutput(CodeToChannel(InputRelays));
		// Automatic reset
		InputRelays = 0;
		StageSelect = false;
	}

	// Feedback
	FeedbackRelays = ((~InputRelays) & 0377);
}

int SwitchSelector::GetFeedback()
{
	if (StageSelect)
	{
		return FeedbackRelays;
	}
	return 0;
}

int SwitchSelector::CodeToChannel(int code) const
{
	if (code >= 1 && code <= 255)
	{
		if (code > 127)
		{
			// Complement of code is the same channel as the code itself
			code = ((~code) & 0377);
		}
		return Code[code - 1];
	}
	return 0;
}