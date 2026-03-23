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
	// Gaussian interpolation filter (FIR with Gaussian kernel).
	//
	// Each coefficient:  g[n] = exp( -0.5 * ((n - M) / sigma)^2 )
	// then normalised to unity DC gain.
	//
	// sigma controls the smoothing width in samples:
	//   small sigma  -> narrow bell -> less smoothing (more transient detail)
	//   large sigma  -> wide bell   -> heavy smoothing (more DMC step rounding)
	//
	// The SetDmcFilterCutoff(sigmaHz, sampleRateHz) interface is preserved:
	//   sigma_samples = sampleRateHz / sigmaHz
	// so callers can keep thinking in Hz while the kernel works in samples.
	//
	// kGaussTaps must be a power of 2 for the ring-buffer AND-wrap trick.
	// Increase to 32 for heavier low-end smoothing at low sigma values.
	// ---------------------------------------------------------------------------
	static constexpr int kGaussTaps = 16;   // power-of-2

	double  _gaussSigmaHz     = 800.0;      // stored as Hz; converted on rebuild
	double  _gaussSampleRate  = 48000.0;
	bool    _dmcFilterEnabled = true;

	bool    _gaussDirty       = true;
	double  _gaussCoeffs[kGaussTaps] = {};

	uint8_t _gaussBuf[kGaussTaps] = {};     // ring buffer of recent raw DMC outputs
	int     _gaussBufHead     = 0;          // oldest entry / next write position

	// Out-of-line: rebuilds _gaussCoeffs from current sigma / sample-rate
	void _RebuildGauss();

	// Push `raw` into the ring buffer then convolve with _gaussCoeffs
	inline uint8_t _ApplyLpf(uint8_t raw)
	{
		_gaussBuf[_gaussBufHead] = raw;
		_gaussBufHead = (_gaussBufHead + 1) & (kGaussTaps - 1);

		if(_gaussDirty) _RebuildGauss();

		if(!_dmcFilterEnabled) return raw;

		double acc = 0.0;
		for(int i = 0; i < kGaussTaps; ++i) {
			const int idx = (_gaussBufHead + i) & (kGaussTaps - 1);
			acc += _gaussCoeffs[i] * static_cast<double>(_gaussBuf[idx]);
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
	// cutoffHz is repurposed as sigmaHz: sigma_samples = sampleRateHz / cutoffHz
	inline void SetDmcFilterCutoff(double cutoffHz, double sampleRateHz)
	{
		_gaussSigmaHz    = cutoffHz;
		_gaussSampleRate = sampleRateHz;
		_gaussDirty      = true;
	}
	inline void EnableDmcFilter(bool enabled)
	{
		_dmcFilterEnabled = enabled;
		_gaussDirty       = true;
	}

	ApuDmcState GetState();
};
