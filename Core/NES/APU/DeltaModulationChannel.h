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

	// Applies the low-pass filter to `raw` (0..127), updates _dmcLpfY,
	// and returns the filtered value.  Returns `raw` unchanged when the
	// filter is disabled or the coefficient hasn't been computed yet.
	inline uint8_t _ApplyLpf(uint8_t raw)
	{
		_EnsureDmcCoeff();
		if(!_dmcFilterEnabled || _dmcLpfA <= 0.0) return raw;
		const double x = static_cast<double>(raw) / 127.0;
		_dmcLpfY += _dmcLpfA * (x - _dmcLpfY);
		int y = static_cast<int>(std::lround(_dmcLpfY * 127.0));
		if(y < 0)   y = 0;
		if(y > 127) y = 127;
		return static_cast<uint8_t>(y);
	}

	// low pass filter state
	// lower cutoff = smooth/quieter / higher cutoff = more noise/louder
	static constexpr double kPI = 3.14159265358979323846;
	double _dmcLpfY = 0.0;                 // filter state
	double _dmcLpfA = -1.0;                // coeff (init state)
	double _dmcLpfCutoff = 800.0;         // filter out freq (hz)
	double _dmcLpfSampleRate = 48000.0;    // output rate 
	bool   _dmcFilterEnabled = true;

	inline void _EnsureDmcCoeff()
	{
		if(!_dmcFilterEnabled) { _dmcLpfA = 0.0; return; }
		if(_dmcLpfA >= 0.0) { return; } // already computed

		if(_dmcLpfCutoff <= 0.0 || _dmcLpfSampleRate <= 0.0) {
			_dmcLpfA = 0.0; // bypass
			return;
		}
		const double x = -2.0 * kPI * _dmcLpfCutoff / _dmcLpfSampleRate;
		double a = 1.0 - std::exp(x);
		if(a < 0.0) a = 0.0;
		if(a > 1.0) a = 1.0;
		_dmcLpfA = a;
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
		_dmcLpfCutoff = cutoffHz;
		_dmcLpfSampleRate = sampleRateHz;
		_dmcLpfA = -1.0;        // force recompute on next GetOutput() func 
	}
	inline void EnableDmcFilter(bool enabled)
	{
		_dmcFilterEnabled = enabled;
		_dmcLpfA = -1.0;        // force recompute
	}

	ApuDmcState GetState();
};
