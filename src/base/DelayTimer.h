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

struct DelayTimerState
{
	double Running;
	double seconds;
};

class DelayTimer {

public:
	DelayTimer(double delay);

	virtual void Timestep(double simdt);
	virtual void SetTime(double t);
	double GetTime();

	void Reset();
	void SetRunning(bool run) { Running = run; };
	bool IsRunning() { return Running; };
	void SetContact(bool cont) { Contact = cont; };
	bool ContactClosed() { return Contact; };

	void SetDelay(double del) { delay = del; }

	void SetState(const DelayTimerState& state);
	void GetState(DelayTimerState& state);

protected:
	double seconds;
	double delay;

	bool Running;
	bool Contact;
};