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

class Saturn;
struct ThrustSpec;

class J2Engine
{
public:
	J2Engine(Saturn* v, ThrustSpec* f1);
	void Timestep(double simdt);

	void SetThrusterIsp(double isp);
	void SetThrusterMax0(double thrust);

	void SetEngineReadyBypass() { EngineReadyBypass = true; }
	void ResetEngineReadyBypass() { EngineReadyBypass = false; }
	void SetEngineStart() { EngineStart = true; }
	void ResetEngineStart() { EngineStart = false; }
	void SetLVDCEngineCutoff() { LVDCEngineStopRelay = true; }
	void ResetLVDCEnginesCutoff() { LVDCEngineStopRelay = false; }
	void SetEDSCutoff() { EDSCutoff = true; }
	void ResetEDSCutoff() { EDSCutoff = false; }
	void SetThrusterDir(double beta_y, double beta_p);
	void SetFailed(bool fail) { EngineFailed = fail; }

	bool GetThrustOK() { return ThrustOK; }
	bool GetFailed() { return EngineFailed; }
protected:
	ThrustSpec* th_j2;
	Saturn* vessel;

	bool EngineReady;
	bool EngineReadyBypass;
	bool EngineStart;
	bool EngineStop;
	bool LVDCEngineStopRelay;
	bool EDSCutoff;
	bool RSSCutoff;

	bool ThrustOK;
	bool EngineRunning;

	bool EngineFailed;

	double ThrustTimer;
	double ThrustLevel;
};