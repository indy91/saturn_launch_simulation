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

#include "LVDC.h"
#include "LVDA.h"
#include "../../util/Constants.h"
#include "../../util/Definitions.h"
#include <string>
#include <fstream>

LVDC::LVDC(LVDA* l) : lvda(l)
{
	KSSINDXTB7A = KSSINDXTB6A = KSSINDXTB6B = KSSINDXTB6C = KSSINDXTB3A = KSSINDXTB5A = KSSINDXTB5B = KSSINDXSIVA = KSSINDXSIVB = KSSINDXS4C1 = KSSINDXGSS = 0;
	KSSINDXSBLO = KSSINDXSBHI = KSSINDXSBOM = KSSINDXECSV = KSSINDXECS1 = KSSINDXGAIN = KSSINDXTB6D = KSSINDXALU = 0;
	for (int i = 0; i < 9; i++)
	{
		SSTTBPTR[i] = 0;
	}
	Timebase = -1;
	SSCounter = 0;
	TBREF = 0.0;
	AttitudeError = _V(0, 0, 0);
	DesiredAtt = _V(0, 0, 0);

	// PITCH POLYNOMIAL (Apollo 11)
	Fx[1][0] = 0.104707442e1;
	Fx[1][1] = -0.147669484;
	Fx[1][2] = 0.824109168e-2;
	Fx[1][3] = -0.403816898e-3;
	Fx[1][4] = 0.338751980e-5;
	Fx[2][0] = -0.687116080e1;
	Fx[2][1] = 0.664009046;
	Fx[2][2] = -0.231195092e-1;
	Fx[2][3] = 0.137310354e-3;
	Fx[2][4] = -0.124330390e-6;
	Fx[3][0] = -0.663318417e1;
	Fx[3][1] = 0.810101295;
	Fx[3][2] = -0.299134054e-1;
	Fx[3][3] = 0.239665250e-3;
	Fx[3][4] = -0.635979551e-6;
	Fx[4][0] = 0.425287982e2;
	Fx[4][1] = -0.125455452e1,
		Fx[4][2] = 0.237693395e-2,
		Fx[4][3] = 0.163508094e-4;
	Fx[4][4] = -0.587090259e-7;
	T_S1 = 33.6;							// Pitch Polynomial Segment Times
	T_S2 = 68.6;							// dto.
	T_S3 = 95.6;							// dto.
	T_ar = 160.0;
}

LVDC::~LVDC()
{

}

void LVDC::Init()
{
	ReadFlightSequenceProgram("../src/base/IU/AS-507 Flight Sequence Program.txt");
}

void LVDC::Timestep(double misst)
{
	// Logic for timebases
	switch (Timebase)
	{
	case -1:
		if (lvda->GetGuidanceReferenceRelease())
		{
			Timebase = 0;

			// Firing commit enable
			int itemp = lvda->GetPIO(016);
			itemp = itemp | 0100000000;

			lvda->WritePIO(016, itemp);
		}
		break;
	case 0:
		if (lvda->GetLiftoff())
		{
			Timebase = 1;
			SSCounter = SSTTBPTR[0];
			TBREF = misst;
		}
		break;
	case 1:
		if ((misst - TBREF) > 134.6)
		{
			Timebase = 2;
			SSCounter = SSTTBPTR[1];
			TBREF = misst;
		}
		break;
	case 2:
		if (lvda->GetSIPropellantDepletionEngineCutoff())
		{
			Timebase = 3;
			SSCounter = SSTTBPTR[2];
			TBREF = misst;
		}
		break;
	case 3:
		if (lvda->GetSIIPropellantDepletionEngineCutoff())
		{
			Timebase = 4;
			SSCounter = SSTTBPTR[3];
			TBREF = misst;
		}
		break;
	case 4:
		if ((misst - TBREF) > 10.0 && lvda->GetSIVBEngineOutA())
		{
			Timebase = 5;
			SSCounter = SSTTBPTR[4];
			TBREF = misst;
		}
		break;
	}

	// Logic for switch selector
	if (Timebase > 0)
	{
		double DT = misst - TBREF;

		SwitchSelectorProcessing(DT);
	}
	if (Timebase < 1) return;

	// Attitude control
	VECTOR3 Att, DAtt;
	double A1, A2, A3, A4, A5, A6;

	Att = lvda->GetAttitude();

	if (Timebase == 0)
	{
		DesiredAtt = _V(0, 0, 0);
	}
	else if (Timebase == 1)
	{
		// Pitch maneuver

		double t_c = (misst - TBREF);

		// Roll, yaw zero
		DesiredAtt.x = 0.0;
		DesiredAtt.z = 0.0;

		if (t_c <= T_ar)
		{
			double dT_F = 0.0;

			int x = 0, y = 0;
			double Tc = t_c - dT_F, cmd = 0;
			if (Tc < T_S1) { x = 1; }
			if (T_S1 <= Tc && Tc < T_S2) { x = 2; }
			if (T_S2 <= Tc && Tc < T_S3) { x = 3; }
			if (T_S3 <= Tc) { x = 4; }
			while (y <= 4) {
				cmd += (Fx[x][y] * ((double)pow(Tc, y)));
				y++;
			}
			DesiredAtt.y = cmd * MathConstants::RAD_PER_DEG;
			if (DesiredAtt.y < 0.0)
			{
				DesiredAtt.y += MathConstants::PI2;
			}
		}
	}
	else if (Timebase == 2)
	{
		DesiredAtt.y = 295.0 * MathConstants::RAD_PER_DEG;
	}
	else if (Timebase == 3)
	{
		DesiredAtt.y = 285.0 * MathConstants::RAD_PER_DEG;
	}
	else if (Timebase == 4)
	{
		DesiredAtt.y = 255.0 * MathConstants::RAD_PER_DEG;

		int disc = lvda->GetPIO(057);

		if (disc & 01000000)
		{
			// S-IVB engine off
			SwitchSelectorCommand cmd;

			cmd.Value = 0;

			cmd.Bits.FlightCommand = 0115;
			cmd.Bits.Read = 1;
			cmd.Bits.Reset = 0;
			cmd.Bits.StageSelect = 010;

			lvda->PIOProcessing(036, cmd.Value);
		}
	}

	A1 = cos(Att.x) * cos(Att.z);
	A2 = sin(Att.x);
	A3 = sin(Att.z);
	A4 = sin(Att.x) * cos(Att.z);
	A5 = cos(Att.x);
	A6 = 1.0;

	DAtt = Att - DesiredAtt;
	for (int i = 0; i < 3; i++)
	{
		if (DAtt.data[i] > MathConstants::PI)
		{
			DAtt.data[i] -= MathConstants::PI2;
		}
		else if (DAtt.data[i] < -MathConstants::PI)
		{
			DAtt.data[i] += MathConstants::PI2;
		}
	}

	AttitudeError.z = A5 * DAtt.z + A4 * DAtt.y; // Yaw
	AttitudeError.y = A1 * DAtt.y + A2 * DAtt.z; // Pitch
	AttitudeError.x = A6 * DAtt.x + A3 * DAtt.y; // Roll

	int AttScaled, num;

	double MSK6 = 1.5 * MathConstants::RAD_PER_DEG;

	// Convert
	for (int i = 0; i < 3; i++)
	{
		if (abs(AttitudeError.data[i]) > MSK6)
		{
			if (AttitudeError.data[i] > 0.0)
			{
				AttitudeError.data[i] = MSK6;
			}
			else
			{
				AttitudeError.data[i] = -MSK6;
			}
		}

		AttScaled = (double)(AttitudeError.data[i] / (0.06 * MathConstants::RAD_PER_DEG));
		if (AttScaled < 0)
		{
			AttScaled = (AttScaled & 0777); // ((~AttScaled) & 0777);
		}
		if (i == 0)
		{
			num = 0146;
		}
		else if (i == 1)
		{
			num = 0152;
		}
		else
		{
			num = 0156;
		}
		lvda->WritePIO(num, AttScaled);
	}
}

void LVDC::ReadFlightSequenceProgram(char* fspfile)
{
	SwitchSelectorSet ssset;
	std::string line;
	std::ifstream file(fspfile);

	SSTABLE.clear();

	if (file.is_open())
	{
		while (getline(file, line))
		{
			if (line.compare("END") == 0)
			{
				break;
			}
			else if ((line.substr(0, 3)).compare("SST") == 0)
			{
				if (line.compare("SSTTB1") == 0)
				{
					SSTTBPTR[0] = SSTABLE.size();
				}
				else if (line.compare("SSTTB2") == 0)
				{
					SSTTBPTR[1] = SSTABLE.size();
				}
				else if (line.compare("SSTTB3") == 0)
				{
					SSTTBPTR[2] = SSTABLE.size();
				}
				else if (line.compare("SSTTB4") == 0)
				{
					SSTTBPTR[3] = SSTABLE.size();
				}
				else if (line.compare("SSTTB5") == 0)
				{
					SSTTBPTR[4] = SSTABLE.size();
				}
				else if (line.compare("SSTTB6") == 0)
				{
					SSTTBPTR[5] = SSTABLE.size();
				}
				else if (line.compare("SSTTB7") == 0)
				{
					SSTTBPTR[6] = SSTABLE.size();
				}
				else if (line.compare("SSTTB8") == 0)
				{
					SSTTBPTR[7] = SSTABLE.size();
				}
				else if (line.compare("SSTTB3A") == 0)
				{
					KSSINDXTB3A = SSTABLE.size();
				}
				else if (line.compare("SSTS4C1") == 0)
				{
					KSSINDXS4C1 = SSTABLE.size();
				}
				else if (line.compare("SSTTB4A") == 0)
				{
					SSTTBPTR[8] = SSTABLE.size();
				}
				else if (line.compare("SSTTB6A") == 0)
				{
					KSSINDXTB6A = SSTABLE.size();
				}
				else if (line.compare("SSTTB6B") == 0)
				{
					KSSINDXTB6B = SSTABLE.size();
				}
				else if (line.compare("SSTTB6C") == 0)
				{
					KSSINDXTB6C = SSTABLE.size();
				}
				else if (line.compare("SSTTB6D") == 0)
				{
					KSSINDXTB6D = SSTABLE.size();
				}
				else if (line.compare("SSTGSS") == 0)
				{
					KSSINDXGSS = SSTABLE.size();
				}
				else if (line.compare("SSTSIVA") == 0)
				{
					KSSINDXSIVA = SSTABLE.size();
				}
				else if (line.compare("SSTSIVB") == 0)
				{
					KSSINDXSIVB = SSTABLE.size();
				}
				else if (line.compare("SSTECS1") == 0)
				{
					KSSINDXECS1 = SSTABLE.size();
				}
				else if (line.compare("SSTECSV") == 0)
				{
					KSSINDXECSV = SSTABLE.size();
				}
				else if (line.compare("SSTSBHI") == 0)
				{
					KSSINDXSBHI = SSTABLE.size();
				}
				else if (line.compare("SSTSBLO") == 0)
				{
					KSSINDXSBLO = SSTABLE.size();
				}
				else if (line.compare("SSTSBOM") == 0)
				{
					KSSINDXSBOM = SSTABLE.size();
				}
				else if (line.compare("SSTTB5A") == 0)
				{
					KSSINDXTB5A = SSTABLE.size();
				}
				else if (line.compare("SSTTB5B") == 0)
				{
					KSSINDXTB5B = SSTABLE.size();
				}
				else if (line.compare("SSTGAIN") == 0)
				{
					KSSINDXGAIN = SSTABLE.size();
				}
				else if (line.compare("SSTALU") == 0)
				{
					KSSINDXALU = SSTABLE.size();
				}
				else if (line.compare("SSTTB7A") == 0)
				{
					KSSINDXTB7A = SSTABLE.size();
				}
			}
			else if (sscanf(line.c_str(), "%lf,%d,%d", &ssset.time, &ssset.stage, &ssset.channel) == 3)
			{
				SSTABLE.push_back(ssset);
			}
		}
	}

	file.close();
}

void LVDC::SwitchSelectorProcessing(double LVDC_TB_ETime)
{
	if (SSTABLE[SSCounter].time < 0.0) return;

	while (LVDC_TB_ETime > SSTABLE[SSCounter].time)
	{
		int feedback = lvda->GetPIO(077);

		SwitchSelectorSet set = SSTABLE[SSCounter];
		SwitchSelectorCommand cmd;
		cmd.Value = 0;

		// Hung stage test
		if (feedback != 0)
		{
			// Issue forced reset
			cmd.Bits.Reset = 1;
			lvda->PIOProcessing(036, cmd.Value);

			// Issue stage and address
			cmd.Bits.Reset = 0;
			cmd.Bits.StageSelect = set.stage;
			cmd.Bits.FlightCommand = set.channel;
			lvda->PIOProcessing(036, cmd.Value);
		}
		else
		{
			// Issue stage and address
			cmd.Bits.StageSelect = set.stage;
			cmd.Bits.FlightCommand = set.channel;
			lvda->PIOProcessing(036, cmd.Value);
		}
		// Verify address
		feedback = lvda->GetPIO(077);
		int feedback_compl = ((~feedback) & 0377);

		if (feedback_compl != set.channel)
		{
			// Issue forced reset
			cmd.Value = 0;
			cmd.Bits.Reset = 1;
			lvda->PIOProcessing(036, cmd.Value);

			// Issue stage and complement address
			cmd.Bits.StageSelect = set.stage;
			cmd.Bits.FlightCommand = ((~set.channel) & 0377);
			lvda->PIOProcessing(036, cmd.Value);
		}
		// Issue read
		cmd.Bits.Read = 1;
		cmd.Bits.Reset = 0;
		lvda->PIOProcessing(036, cmd.Value);
		// Reset read

		//lvda->SwitchSelector(SSTABLE[SSCounter].stage, SSTABLE[SSCounter].channel);
		//fprintf(lvlog, "[TB%d+%f] Switch Selector command issued: Stage %d Channel %d\r\n", LVDC_Timebase, LVDC_TB_ETime, table[CommandSequence].stage, table[CommandSequence].channel);
		SSCounter++;
		if (SSTABLE[SSCounter].time < 0.0) return;
	}
}

LVDC1B::LVDC1B(LVDA* l) : LVDC(l)
{

}

LVDCSV::LVDCSV(LVDA* l) : LVDC(l)
{

}