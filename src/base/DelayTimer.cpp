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

#include "DelayTimer.h"

DelayTimer::DelayTimer(double del)

{
	Running = false;

	delay = del;
	seconds = 0.0;
	Contact = false;
}

double DelayTimer::GetTime()

{
	return seconds;
}

void DelayTimer::SetTime(double t)

{
	if (seconds > delay) {
		SetContact(true);
		return;
	}

	seconds = t;
}

void DelayTimer::Timestep(double simdt)
{

	if (Running) {
		double t = GetTime();

		t += simdt;

		SetTime(t);

		Running = false;
	}
	else
	{
		Reset();
	}
}

void DelayTimer::Reset()

{
	seconds = 0.0;
	Running = false;
	Contact = false;
}

void DelayTimer::SetState(const DelayTimerState& state)
{
	Running = state.Running;
	SetTime(state.seconds);
}

void DelayTimer::GetState(DelayTimerState& state)
{
	state.Running = Running;
	state.seconds = GetTime();
}