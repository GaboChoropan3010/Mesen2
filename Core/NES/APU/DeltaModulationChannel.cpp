#include "pch.h"

#include "NES/APU/DeltaModulationChannel.h"
#include "NES/APU/NesApu.h"
#include "NES/NesCpu.h"
#include "NES/NesConstants.h"
#include "NES/NesConsole.h"
#include "NES/NesMemoryManager.h"

DeltaModulationChannel::DeltaModulationChannel(NesConsole* console)
	: _timer(AudioChannel::DMC, console->GetSoundMixer())
{
	_console = console;

	// lfp state initilization
	_dmcLpfY = static_cast<double>(_outputLevel) / 127.0;
	_dmcLpfA = -1.0; // lazy compute on first use
}

void DeltaModulationChannel::Reset(bool softReset)
{
	_timer.Reset(softReset);

	if(!softReset) {
		_sampleAddr = 0xC000;
		_sampleLength = 1;
	}

	_outputLevel = 0;
	_irqEnabled = false;
	_loopFlag = false;

	_currentAddr = 0;
	_bytesRemaining = 0;
	_readBuffer = 0;
	_bufferEmpty = true;

	_shiftRegister = 0;
	_bitsRemaining = 8;
	_silenceFlag = true;
	_needToRun = false;
	_transferStartDelay = 0;
	_disableDelay = 0;

	_lastValue4011 = 0;

	// initialize filter state on reset
	_dmcLpfY = static_cast<double>(_outputLevel) / 127.0;
	_dmcLpfA = -1.0; // recompute next use

	_timer.SetPeriod(
		 (NesApu::GetApuRegion(_console) == ConsoleRegion::Ntsc
			 ? _dmcPeriodLookupTableNtsc
			 : _dmcPeriodLookupTablePal)[0] -
		 1);

	_timer.SetTimer(_timer.GetPeriod());
}

void DeltaModulationChannel::InitSample()
{
	_currentAddr = _sampleAddr;
	_bytesRemaining = _sampleLength;
	_needToRun |= _bytesRemaining > 0;
}

void DeltaModulationChannel::StartDmcTransfer()
{
	if(_bufferEmpty && _bytesRemaining > 0) {
		_console->GetCpu()->StartDmcTransfer();
	}
}

uint16_t DeltaModulationChannel::GetDmcReadAddress()
{
	return _currentAddr;
}

void DeltaModulationChannel::SetDmcReadBuffer(uint8_t value)
{
	if(_bytesRemaining > 0) {
		_readBuffer = value;
		_bufferEmpty = false;

		_currentAddr++;
		if(_currentAddr == 0) {
			_currentAddr = 0x8000;
		}

		_bytesRemaining--;

		if(_bytesRemaining == 0) {
			if(_loopFlag) {
				InitSample();
			} else if(_irqEnabled) {
				_console->GetCpu()->SetIrqSource(IRQSource::DMC);
			}
		}
	}

	if(_sampleLength == 1 && !_loopFlag) {
		if(_bitsRemaining == 8 && _timer.GetTimer() == _timer.GetPeriod() &&
			_console->GetNesConfig().EnableDmcSampleDuplicationGlitch) {
			_shiftRegister = _readBuffer;
			_silenceFlag = false;
			_bufferEmpty = true;
			InitSample();
			StartDmcTransfer();
		} else if(_bitsRemaining == 1 && _timer.GetTimer() < 2) {
			_shiftRegister = _readBuffer;
			_bufferEmpty = false;
			InitSample();
			_disableDelay = 3;
		}
	}
}

void DeltaModulationChannel::Run(uint32_t targetCycle)
{
	while(_timer.Run(targetCycle)) {
		if(!_silenceFlag) {
			uint8_t bit;
			if(_console->GetNesConfig().ReverseDpcmBitOrder) {
				bit = _shiftRegister & 0x80;
				_shiftRegister <<= 1;
			} else {
				bit = _shiftRegister & 0x01;
				_shiftRegister >>= 1;
			}

			if(bit) {
				if(_outputLevel <= 125) _outputLevel += 2;
			} else {
				if(_outputLevel >= 2) _outputLevel -= 2;
			}
		}

		_bitsRemaining--;
		if(_bitsRemaining == 0) {
			_bitsRemaining = 8;
			if(_bufferEmpty) {
				_silenceFlag = true;
			} else {
				_silenceFlag = false;
				_shiftRegister = _readBuffer;
				_bufferEmpty = true;
				_needToRun = true;
				if(_transferStartDelay == 0) {
					StartDmcTransfer();
				}
			}
		}

		// apply lowpass filter before mixing
		_EnsureDmcCoeff();

		uint8_t outByte = _outputLevel; // 0..127 raw
		if(_dmcFilterEnabled && _dmcLpfA > 0.0) {
			const double x = static_cast<double>(outByte) / 127.0;
			_dmcLpfY += _dmcLpfA * (x - _dmcLpfY);
			int yInt = static_cast<int>(std::lround(_dmcLpfY * 127.0));
			if(yInt < 0) yInt = 0;
			if(yInt > 127) yInt = 127;
			outByte = static_cast<uint8_t>(yInt);
		}

		_timer.AddOutput(outByte);
	}
}

bool DeltaModulationChannel::IrqPending(uint32_t cyclesToRun)
{
	if(_irqEnabled && _bytesRemaining > 0) {
		uint32_t cyclesToEmptyBuffer =
			(_bitsRemaining + (_bytesRemaining - 1) * 8) * _timer.GetPeriod();
		if(cyclesToRun >= cyclesToEmptyBuffer) {
			return true;
		}
	}
	return false;
}

bool DeltaModulationChannel::GetStatus()
{
	return _bytesRemaining > 0;
}

void DeltaModulationChannel::GetMemoryRanges(MemoryRanges& ranges)
{
	ranges.AddHandler(MemoryOperation::Write, 0x4010, 0x4013);
}

void DeltaModulationChannel::WriteRam(uint16_t addr, uint8_t value)
{
	_console->GetApu()->Run();

	switch(addr & 0x03) {
		case 0: // 4010
			_irqEnabled = (value & 0x80) != 0;
			_loopFlag = (value & 0x40) != 0;

			_timer.SetPeriod(
				 (NesApu::GetApuRegion(_console) == ConsoleRegion::Ntsc
					 ? _dmcPeriodLookupTableNtsc
					 : _dmcPeriodLookupTablePal)[value & 0x0F] -
				 1);

			if(!_irqEnabled) {
				_console->GetCpu()->ClearIrqSource(IRQSource::DMC);
			}
			break;

		case 1:
		{ // 4011
			uint8_t newValue = value & 0x7F;
			uint8_t previousLevel = _outputLevel;
			_outputLevel = newValue;

			if(_console->GetNesConfig().ReduceDmcPopping &&
				abs(_outputLevel - previousLevel) > 50) {
				_outputLevel -= (_outputLevel - previousLevel) / 2;
			}

			_EnsureDmcCoeff();
			uint8_t outByte = _outputLevel;
			if(_dmcFilterEnabled && _dmcLpfA > 0.0) {
				const double x = static_cast<double>(outByte) / 127.0;
				_dmcLpfY += _dmcLpfA * (x - _dmcLpfY);
				int yInt = static_cast<int>(std::lround(_dmcLpfY * 127.0));
				if(yInt < 0) yInt = 0;
				if(yInt > 127) yInt = 127;
				outByte = static_cast<uint8_t>(yInt);
			}

			_timer.AddOutput(outByte);

			if(_lastValue4011 != value && newValue > 0) {
				_console->SetNextFrameOverclockStatus(true);
			}
			_lastValue4011 = newValue;
			break;
		}

		case 2: // 4012
			_sampleAddr = 0xC000 | ((uint32_t)value << 6);
			if(value > 0) _console->SetNextFrameOverclockStatus(false);
			break;

		case 3: // 4013
			_sampleLength = (value << 4) | 0x0001;
			if(value > 0) _console->SetNextFrameOverclockStatus(false);
			break;
	}
}

void DeltaModulationChannel::EndFrame()
{
	_timer.EndFrame();
}

void DeltaModulationChannel::SetEnabled(bool enabled)
{
	if(!enabled) {
		if(_disableDelay == 0) {
			if((_console->GetCpu()->GetCycleCount() & 0x01) == 0) {
				_disableDelay = 2;
			} else {
				_disableDelay = 3;
			}
		}
		_needToRun = true;
	} else if(_bytesRemaining == 0) {
		InitSample();

		if((_console->GetCpu()->GetCycleCount() & 0x01) == 0) {
			_transferStartDelay = 2;
		} else {
			_transferStartDelay = 3;
		}
		_needToRun = true;
	}
}

void DeltaModulationChannel::ProcessClock()
{
	if(_disableDelay && --_disableDelay == 0) {
		_disableDelay = 0;
		_bytesRemaining = 0;
		_console->GetCpu()->StopDmcTransfer();
	}

	if(_transferStartDelay && --_transferStartDelay == 0) {
		StartDmcTransfer();
	}

	_needToRun = _disableDelay || _transferStartDelay || _bytesRemaining;
}

bool DeltaModulationChannel::NeedToRun()
{
	if(_needToRun) {
		ProcessClock();
	}
	return _needToRun;
}

ApuDmcState DeltaModulationChannel::GetState()
{
	ApuDmcState state;
	state.BytesRemaining = _bytesRemaining;
	state.IrqEnabled = _irqEnabled;
	state.Loop = _loopFlag;
	state.OutputVolume = _timer.GetLastOutput();
	state.Period = _timer.GetPeriod();
	state.Timer = _timer.GetTimer();
	state.SampleRate =
		(double)NesConstants::GetClockRate(NesApu::GetApuRegion(_console)) /
		(_timer.GetPeriod() + 1);
	state.SampleAddr = _sampleAddr;
	state.NextSampleAddr = _currentAddr;
	state.SampleLength = _sampleLength;
	return state;
}

void DeltaModulationChannel::Serialize(Serializer& s)
{
	SV(_sampleAddr);
	SV(_sampleLength);
	SV(_outputLevel);
	SV(_irqEnabled);
	SV(_loopFlag);
	SV(_currentAddr);
	SV(_bytesRemaining);
	SV(_readBuffer);
	SV(_bufferEmpty);
	SV(_shiftRegister);
	SV(_bitsRemaining);
	SV(_silenceFlag);
	SV(_needToRun);
	SV(_timer);

	SV(_transferStartDelay);
	SV(_disableDelay);
}
