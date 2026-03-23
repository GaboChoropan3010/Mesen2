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

	// ---------------------------------------------------------------------------
	// Band-limited low-pass filter: windowed-sinc FIR with Lanczos-2 window.
	//
	// Design parameters
	//   kFirTaps  – number of FIR coefficients (must be a power of 2 so the
	//               ring-buffer index wrap is a single AND)
	//   kLanczosA – Lanczos window lobe count (2 = Lanczos-2, standard choice)
	//
	// Coefficients are rebuilt lazily in _RebuildFir() whenever cutoff or
	// sample-rate changes (_firDirty = true).  Per-sample cost is kFirTaps
	// MACs on a ring buffer of uint8 inputs.
	// ---------------------------------------------------------------------------
	static constexpr double kPI       = 3.14159265358979323846;
	static constexpr int    kFirTaps  = 16;   // power-of-2; raise for steeper roll-off
	static constexpr int    kLanczosA = 2;

	double  _firCutoffHz      = 800.0;
	double  _firSampleRate    = 48000.0;
	bool    _dmcFilterEnabled = true;

	bool    _firDirty         = true;
	double  _firCoeffs[kFirTaps] = {};

	uint8_t _firBuf[kFirTaps] = {};  // ring buffer of recent raw DMC outputs
	int     _firBufHead       = 0;   // index of the oldest entry (write position)

	// Lanczos window kernel – defined inline, no state
	static inline double _Lanczos(double x, int a)
	{
		if(x == 0.0)              return 1.0;
		if(x <= -a || x >= a)     return 0.0;
		const double pix  = kPI * x;
		const double pixa = kPI * x / a;
		return (std::sin(pix) / pix) * (std::sin(pixa) / pixa);
	}

	// Out-of-line: rebuilds _firCoeffs from current cutoff/rate parameters
	void _RebuildFir();

	// Push `raw` into the ring buffer then convolve with _firCoeffs
	inline uint8_t _ApplyLpf(uint8_t raw)
	{
		_firBuf[_firBufHead] = raw;
		_firBufHead = (_firBufHead + 1) & (kFirTaps - 1);

		if(_firDirty) _RebuildFir();

		if(!_dmcFilterEnabled) return raw;

		double acc = 0.0;
		for(int i = 0; i < kFirTaps; ++i) {
			const int idx = (_firBufHead + i) & (kFirTaps - 1);
			acc += _firCoeffs[i] * static_cast<double>(_firBuf[idx]);
		}

		int y = static_cast<int>(std::lround(acc));
		if(y < 0)   y = 0;
		if(y > 127) y = 127;
		return static_cast<uint8_t>(y);
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

	// filtered output; filter is applied in Run() and WriteRam() before
	// AddOutput(), so GetLastOutput() already carries the filtered value.
	uint8_t GetOutput()
	{
		return _timer.GetLastOutput();
	}

	// control (can be called from apu init or ui)
	inline void SetDmcFilterCutoff(double cutoffHz, double sampleRateHz)
	{
		_firCutoffHz   = cutoffHz;
		_firSampleRate = sampleRateHz;
		_firDirty      = true;   // recompute coefficients on next _ApplyLpf call
	}
	inline void EnableDmcFilter(bool enabled)
	{
		_dmcFilterEnabled = enabled;
		_firDirty         = true;
	}

	ApuDmcState GetState();
};
