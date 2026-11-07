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

namespace MathConstants
{
	//Math constants
	static const double PI = 3.14159265358979323846264338327950288419716939937511;
	static const double PI2 = 6.28318530717958647692528676655900576839433879875022;
	static const double PI05 = 1.57079632679489661923132169163975144209858469968756;

	//Angle conversion
	static const double RAD_PER_DEG =
		3.14159265358979323846264338327950288419716939937511 / 180.0;
	static const double DEG_PER_RAD =
		180.0 / 3.14159265358979323846264338327950288419716939937511;

	static const double ARCSEC_PER_DEGREE = 3600.0;
	static const double DEG_PER_ARCSEC = 1.0 / 3600.0;
	static const double RAD_PER_ARCSEC = DEG_PER_ARCSEC * RAD_PER_DEG;

	//Mass conversion
	static const double LBM_TO_KG = 0.45359237;
	static const double SLUG_TO_KG = 14.59390294;

	//Force conversion
	static const double LBF_TO_N = 4.4482216152605;

	//Length
	static const double INCH_TO_M = 0.0254;
	static const double FOOT_TO_M = 0.3048;
	static const double STATUTE_MILE_TO_M = 1609.344;
	static const double NAUTICAL_MILE_TO_M = 1852.0;
	static const double M_TO_KM = 0.001;
	static const double KM_TO_M = 1000.0;
}

namespace GmatTimeConstants
{
	const double SECS_PER_DAY = 86400.0;
	const double SECS_PER_HOUR = 3600.0;
	const double SECS_PER_MINUTE = 60.0;

}