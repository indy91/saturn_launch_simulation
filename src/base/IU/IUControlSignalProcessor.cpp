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

#include "iu.h"
#include "IUControlSignalProcessor.h"
#include "../../util/Constants.h"

IUControlSignalProcessor::IUControlSignalProcessor(IU* iu)
{
	this->iu = iu;
	PitchNo1Overrate = false;
	PitchNo2Overrate = false;
	PitchNo3Overrate = false;
	YawNo1Overrate = false;
	YawNo2Overrate = false;
	YawNo3Overrate = false;
	RollNo1Overrate = false;
	RollNo2Overrate = false;
	RollNo3Overrate = false;
	PYLimit = 0.0;
	AttRate = _V(0, 0, 0);
}

void IUControlSignalProcessor::Timestep()
{
	//Signal "A"
	if (iu->GetControlDistributor()->GetExcessiveRatePYRAutoAbortInhibit() || iu->GetControlDistributor()->GetExcessiveRateRollAutoAbortInhibit())
	{
		PYLimit = 9.2 * MathConstants::RAD_PER_DEG;
	}
	else
	{
		PYLimit = 4.0 * MathConstants::RAD_PER_DEG;
	}

	AttRate = iu->GetLVRG()->GetRates();

	if (abs(AttRate.x) > 20.0 * MathConstants::RAD_PER_DEG)
	{
		RollNo1Overrate = true;
		RollNo2Overrate = true;
		RollNo3Overrate = true;
	}
	else
	{
		RollNo1Overrate = false;
		RollNo2Overrate = false;
		RollNo3Overrate = false;
	}

	if (abs(AttRate.y) > PYLimit)
	{
		PitchNo1Overrate = true;
		PitchNo2Overrate = true;
		PitchNo3Overrate = true;
	}
	else
	{
		PitchNo1Overrate = false;
		PitchNo2Overrate = false;
		PitchNo3Overrate = false;
	}

	if (abs(AttRate.z) > PYLimit)
	{
		YawNo1Overrate = true;
		YawNo2Overrate = true;
		YawNo3Overrate = true;
	}
	else
	{
		YawNo1Overrate = false;
		YawNo2Overrate = false;
		YawNo3Overrate = false;
	}
}