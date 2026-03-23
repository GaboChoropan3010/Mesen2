#pragma once
#include "pch.h"
#include "NES/APU/ApuTimer.h"
#include "NES/INesMemoryHandler.h"
#include "Utilities/ISerializable.h"

#include <cmath>      
#include <cstdint>

// low pass filter, smooth sharp steps, quiet down ("sandpaper" the noise)

class NesConsole;

class DeltaModulationChannel : public INesMemoryHandler, public ISerializable
{
private:
	// dmc period tables 
	static constexpr uint16_t _dmcPeriodLookupTableNtsc[16] = { 428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106,  84,  72,  54 };
	static constexpr uint16_t _dmcPeriodLookupTablePal[16] = { 398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118,  98,  78,  66,  50 };

	NesConsole* _console = nullptr;
	ApuTimer _timer;

	// dmc state
	uint16_t _sampleAddr = 0;
	uint16_t _sampleLength = 0;
	uint8_t  _outputLevel = 0;
	bool     _irqEnabled = false;
	bool     _loopFlag = false;

	uint16_t _currentAddr = 0;
	uint16_t _bytesRemaining = 0;
	uint8_t  _readBuffer = 0;
	bool     _bufferEmpty = true;

	uint8_t  _shiftRegister = 0;
	uint8_t  _bitsRemaining = 0;
	bool     _silenceFlag = true;
	bool     _needToRun = false;
	uint8_t  _disableDelay = 0;
	uint8_t  _transferStartDelay = 0;

	uint8_t  _lastValue4011 = 0;

	void InitSample();

	// -------------------------------------------------------------------------
	// Gaussian pre-filter (anti-aliasing at the blip-buffer layer)
	//
	// WHY HERE:
	//   AddOutput() calls _mixer->AddDelta() which records a hard step impulse
	//   at a precise CPU cycle timestamp into the blip-buffer. The blip-buffer
	//   then integrates those impulses into the audio frame. Any filter applied
	//   AFTER AddOutput (e.g. on _lastOutput) never touches the waveform already
	//   recorded in the buffer — it only affects the $401A debug read. Aliasing
	//   originates at the impulse, so the filter must run BEFORE AddDelta.
	//
	// HOW IT WORKS:
	//   On each DMC timer tick, instead of one AddDelta(±2) call, we spread the
	//   step energy across kGaussTaps evenly-spaced sub-steps within the DMC
	//   period. Each sub-step contributes (weight[i] * delta) to AddDelta at an
	//   interpolated cycle timestamp. The blip-buffer receives a Gaussian ramp
	//   rather than a hard impulse, eliminating the alias before integration.
	//
	//   Weights are precomputed (lazy, _gaussDirty flag). Per-tick cost is
	//   kGaussTaps small AddDelta calls; because AddDelta is a no-op when
	//   delta == 0 and weights are normalised, DC gain is preserved exactly.
	//
	// PARAMETERS:
	//   kGaussTaps  — spread width in sub-steps (power-of-2 not required here)
	//   _gaussSigma — std-dev of the bell in sub-steps; larger = more smoothing
	// -------------------------------------------------------------------------
	static constexpr double kPI        = 3.14159265358979323846;
	static constexpr int    kGaussTaps = 16;

	double _gaussSigma       = 4.0;   // bell width in sub-steps
	bool   _dmcFilterEnabled = true;
	bool   _gaussDirty       = true;
	double _gaussWeights[kGaussTaps] = {};

	void _RebuildGauss();

	inline void _GaussianAddDelta(AudioChannel channel, NesSoundMixer* mixer,
	                               uint32_t periodStart, uint16_t period,
	                               int8_t delta)
	{
		if(!_dmcFilterEnabled || period == 0) {
			mixer->AddDelta(channel, periodStart, delta);
			return;
		}

		if(_gaussDirty) _RebuildGauss();

		const double fDelta  = static_cast<double>(delta);
		const double fPeriod = static_cast<double>(period);
		double remainder = 0.0;

		for(int i = 0; i < kGaussTaps; ++i) {
			uint32_t cycleOffset = static_cast<uint32_t>(
				(static_cast<double>(i) / kGaussTaps) * fPeriod + 0.5);

			double exact  = fDelta * _gaussWeights[i] + remainder;
			int8_t iDelta = static_cast<int8_t>(exact >= 0
			                  ? static_cast<int>(exact + 0.5)
			                  : static_cast<int>(exact - 0.5));
			remainder = exact - static_cast<double>(iDelta);

			if(iDelta != 0)
				mixer->AddDelta(channel, periodStart + cycleOffset, iDelta);
		}

		// flush rounding residual so sum == delta exactly
		int8_t residual = static_cast<int8_t>(
			remainder >= 0 ? static_cast<int>(remainder + 0.5)
			               : static_cast<int>(remainder - 0.5));
		if(residual != 0)
			mixer->AddDelta(channel, periodStart + period, residual);
	}

public:
	DeltaModulationChannel(NesConsole* console);

	void Run(uint32_t targetCycle);

	void Reset(bool softReset);
	void Serialize(Serializer& s) override;

	bool IrqPending(uint32_t cyclesToRun);
	bool NeedToRun();
	bool GetStatus();
	void GetMemoryRanges(MemoryRanges& ranges) override;
	void WriteRam(uint16_t addr, uint8_t value) override;
	uint8_t ReadRam(uint16_t addr) override { return 0; }
	void EndFrame();

	void SetEnabled(bool enabled);
	void ProcessClock();
	void StartDmcTransfer();
	uint16_t GetDmcReadAddress();
	void SetDmcReadBuffer(uint8_t value);

	// GetOutput() returns the last committed level. The Gaussian pre-filter
	// has already shaped the waveform inside the blip-buffer; no post-processing
	// needed here.
	uint8_t GetOutput()
	{
		return _timer.GetLastOutput();
	}

	// sigma: Gaussian bell width in sub-steps (1.0 = very sharp, 6.0+ = heavy smoothing)
	inline void SetDmcFilterSigma(double sigma)
	{
		_gaussSigma  = sigma > 0.0 ? sigma : 1.0;
		_gaussDirty  = true;
	}
	// Legacy shim: callers using SetDmcFilterCutoff(hz, rate) are remapped to sigma.
	// sigma_samples = sampleRate / cutoffHz is a reasonable approximation.
	inline void SetDmcFilterCutoff(double cutoffHz, double sampleRateHz)
	{
		SetDmcFilterSigma(sampleRateHz / (cutoffHz > 0.0 ? cutoffHz : 1.0) / kGaussTaps);
	}
	inline void EnableDmcFilter(bool enabled)
	{
		_dmcFilterEnabled = enabled;
		_gaussDirty       = true;
	}

	ApuDmcState GetState();
};
