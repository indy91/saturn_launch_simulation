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

class LVRG {
public:
	LVRG();                                                                 // Cons
	void Init(IU* v);														// Initialization
	void Timestep(double simdt);                                            // Update function
	VECTOR3 GetRates() { return rates; };

protected:
	VECTOR3 rates;                                                          // Detected rotation acceleration
	IU* iu;																	// Pointer to ship we're attached to
};