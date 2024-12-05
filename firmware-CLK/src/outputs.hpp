#include <Arduino.h>

#include "euclidean.hpp"
#include "utils.hpp"

// Define a type for the DAC output type
enum OutputType {
    DigitalOut = 0,
    DACOut = 1,
};

// Implement WaveformType enum
enum WaveformType {
    Square = 0,
    Triangle,
    Sine,
    Sawtooth,
    Random,
    SmoothRandom,
};
String WaveformTypeDescriptions[] = {"Square", "Triangle", "Sine", "Sawtooth", "Random", "SmoothRdn"};
int WaveformTypeLength = sizeof(WaveformTypeDescriptions) / sizeof(WaveformTypeDescriptions[0]);

class Output {
  public:
    // Constructor
    Output(int ID, OutputType type);

    // Pulse State
    void Pulse(int PPQN, unsigned long tickCounter);
    bool GetPulseState() { return _isPulseOn; }
    void SetPulse(bool state) { _isPulseOn = state; }
    void TogglePulse() { _isPulseOn = !_isPulseOn; }
    bool HasPulseChanged();
    void SetExternalClock(bool state) { _externalClock = state; }
    void IncrementInternalCounter() { _internalPulseCounter++; }

    // Output State
    bool GetOutputState() { return _state; }
    void SetOutputState(bool state) { _state = state; }
    void ToggleOutputState() { _state = !_state; }
    void ToggleMasterState();

    // Divider
    int GetDividerIndex() { return _dividerIndex; }
    void SetDivider(int index) { _dividerIndex = constrain(index, 0, dividerAmount - 1); }
    String GetDividerDescription() { return _dividerDescription[_dividerIndex]; }
    int GetDividerAmounts() { return dividerAmount; }

    // Duty Cycle
    int GetDutyCycle() { return _dutyCycle; }
    void SetDutyCycle(int dutyCycle) { _dutyCycle = constrain(dutyCycle, 1, 99); }
    String GetDutyCycleDescription() { return String(_dutyCycle) + "%"; }

    // Output Level
    int GetLevel() { return _level; }
    int GetOutputLevel(); // Output Level based on the output type
    String GetLevelDescription() { return String(_level) + "%"; }
    void SetLevel(int level) { _level = constrain(level, 0, 100); }

    // Output Offset
    int GetOffset() { return _offset; }
    void SetOffset(int offset) { _offset = constrain(offset, 0, 100); }
    String GetOffsetDescription() { return String(_offset) + "%"; }

    // Swing
    void SetSwingAmount(int swingAmount) { _swingAmountIndex = constrain(swingAmount, 0, 6); }
    int GetSwingAmountIndex() { return _swingAmountIndex; }
    int GetSwingAmounts() { return 7; }
    String GetSwingAmountDescription() { return _swingAmountDescriptions[_swingAmountIndex]; }
    void SetSwingEvery(int swingEvery) { _swingEvery = constrain(swingEvery, 1, 16); }
    int GetSwingEvery() { return _swingEvery; }

    // Pulse Probability
    void SetPulseProbability(int pulseProbability) { _pulseProbability = constrain(pulseProbability, 0, 100); }
    int GetPulseProbability() { return _pulseProbability; }
    String GetPulseProbabilityDescription() { return String(_pulseProbability) + "%"; }

    // Euclidean Rhythm
    void SetEuclidean(bool euclidean);
    void ToggleEuclidean() { SetEuclidean(!_euclideanParams.enabled); }
    bool GetEuclidean() { return _euclideanParams.enabled; }
    int GetRhythmStep(int i) { return _euclideanRhythm[i]; }
    void SetEuclideanSteps(int steps);
    int GetEuclideanSteps() { return _euclideanParams.steps; }
    void SetEuclideanTriggers(int triggers);
    int GetEuclideanTriggers() { return _euclideanParams.triggers; }
    void SetEuclideanRotation(int rotation);
    int GetEuclideanRotation() { return _euclideanParams.rotation; }
    void SetEuclideanPadding(int pad);
    int GetEuclideanPadding() { return _euclideanParams.pad; }

    // Phase
    void SetPhase(int phase) { _phase = constrain(phase, 0, 100); }
    int GetPhase() { return _phase; }
    String GetPhaseDescription() { return String(_phase) + "%"; }

    // Waveform Type
    int GetWaveformTypeIndex() { return int(_waveformType); }
    void SetWaveformType(WaveformType type) { _waveformType = type; }
    WaveformType GetWaveformType() { return _waveformType; }
    String GetWaveformTypeDescription() { return WaveformTypeDescriptions[_waveformType]; }

  private:
    // Constants
    const int MaxDACValue = 4095;
    static int const dividerAmount = 10;
    float _clockDividers[dividerAmount] = {0.03125, 0.0625, 0.125, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0};
    String _dividerDescription[dividerAmount] = {"/32", "/16", "/8", "/4", "/2", "x1", "x2", "x4", "x8", "x16"};
    static int const MaxEuclideanSteps = 64;

    // The shuffle of the TR-909 delays each even-numbered 1/16th by 2/96 of a beat for shuffle setting 1,
    // 4/96 for 2, 6/96 for 3, 8/96 for 4, 10/96 for 5 and 12/96 for 6.
    float _swingAmounts[7] = {0, 2, 4, 6, 8, 10, 12};
    String _swingAmountDescriptions[7] = {"0", "2/96", "4/96", "6/96", "8/96", "10/96", "12/96"};

    // Variables
    int _ID;
    bool _externalClock = false;             // External clock state
    OutputType _outputType;                  // 0 = Digital, 1 = DAC
    int _dividerIndex = 5;                   // Default to 1
    int _dutyCycle = 50;                     // Default to 50%
    int _phase = 0;                          // Phase offset, default to 0% (in phase with master)
    int _level = 100;                        // Output voltage level for DAC outs (Default to 100%)
    int _offset = 0;                         // Output voltage offset for DAC outs (default to 0%)
    bool _isPulseOn = false;                 // Pulse state
    bool _lastPulseState = false;            // Last pulse state
    bool _state = true;                      // Output state
    bool _oldState = true;                   // Previous output state (for master stop)
    bool _masterState = true;                // Master output state
    int _pulseProbability = 100;             // % chance of pulse
    unsigned long _internalPulseCounter = 0; // Pulse counter (used for external clock division)

    // Waveform generation variables
    WaveformType _waveformType = WaveformType::Square; // Default to square wave
    bool _waveActive = false;
    bool _waveDirection = true; // Waveform direction (true = up, false = down)
    float _waveValue = 0.0f;
    float _triangleWaveStep = 0.0f;
    float _sineWaveAngle = 0.0f;

    // Swing variables
    unsigned int _swingAmountIndex = 0; // Swing amount index
    int _swingEvery = 2;                // Swing every x notes

    // Euclidean rhythm variables
    int _euclideanStepIndex = 0;             // Current step in the pattern
    EuclideanParams _euclideanParams;        // Euclidean rhythm parameters
    int _euclideanRhythm[MaxEuclideanSteps]; // Euclidean rhythm pattern

    // Functions
    void StartWaveform();
    void StopWaveform();
    void ResetWaveform();
    void StopWave();
    void GenerateTriangleWave(int);
    void GenerateSineWave(int);
    void GenerateSawtoothWave(int);
    void GenerateRandomWave(int);
    void GenerateSmoothRandomWave(int);
};

// Constructor
Output::Output(int ID, OutputType type) {
    _ID = ID;
    _outputType = type;
    GeneratePattern(_euclideanParams, _euclideanRhythm);
}

// Pulse function
void Output::Pulse(int PPQN, unsigned long globalTick) {
    // If not stopped, generate the pulse
    if (_state) {
        // Calculate the period duration in ticks
        float periodTicks = PPQN / _clockDividers[_dividerIndex];

        // Calculate the phase offset in ticks
        unsigned long phaseOffsetTicks = periodTicks * (_phase / 100.0);

        // Apply swing to the tick counter
        unsigned long tickCounterSwing = globalTick;

        // Calculate the tick counter with swing applied
        if (int(globalTick / periodTicks) % _swingEvery == 0) {
            tickCounterSwing = globalTick - _swingAmounts[_swingAmountIndex];
        }

        // Calculate the clock divider for external clock
        int clockDividerExternal = 1 / _clockDividers[_dividerIndex];

        // Calculate the pulse duration (in ticks) based on the duty cycle
        int _pulseDuration = int(periodTicks * (_dutyCycle / 100.0));
        int _externalPulseDuration = int(clockDividerExternal * (_dutyCycle / 100.0));

        // Lambda function to handle timing
        auto generatePulse = [this]() {
            if (!_euclideanParams.enabled) {
                // If not using Euclidean rhythm, generate waveform based on the pulse probability
                if (random(100) < _pulseProbability) {
                    StartWaveform();
                } else {
                    // We stop the waveform directly if the pulse probability is not met since StopWaveform() is used for the square wave
                    ResetWaveform();
                }
            } else {
                // If using Euclidean rhythm, check if the current step is active
                if (_euclideanRhythm[_euclideanStepIndex] == 1) {
                    StartWaveform();
                } else {
                    ResetWaveform();
                }
                _euclideanStepIndex++;
                // Restart the Euclidean rhythm if it reaches the end
                if (_euclideanStepIndex >= _euclideanParams.steps + _euclideanParams.pad) {
                    _euclideanStepIndex = 0;
                }
            }
        };

        // If using an external clock, generate a pulse based on the internal pulse counter
        // dirty workaround to make this work with clock dividers
        if (_externalClock && _clockDividers[_dividerIndex] < 1) {
            if (_internalPulseCounter % clockDividerExternal == 0 || _internalPulseCounter == 0) {
                generatePulse();
            } else if (_internalPulseCounter % _externalPulseDuration == 0) {
                StopWaveform();
            }
        } else {
            // Handle internal clock timing
            if ((tickCounterSwing - phaseOffsetTicks) % int(periodTicks) == 0 || (globalTick == 0)) {
                generatePulse();
            } else if ((tickCounterSwing - phaseOffsetTicks) % _pulseDuration == 0) {
                StopWaveform();
            }
        }
        // Handle the waveform generation
        switch (_waveformType) {
        case WaveformType::Triangle:
            GenerateTriangleWave(PPQN);
            break;
        case WaveformType::Sine:
            GenerateSineWave(PPQN);
            break;
        case WaveformType::Sawtooth:
            GenerateSawtoothWave(PPQN);
            break;
        case WaveformType::Random:
            GenerateRandomWave(PPQN);
            break;
        case WaveformType::SmoothRandom:
            GenerateSmoothRandomWave(PPQN);
            break;
        default:
            // For square wave or other types
            break;
        }
    } else {
        StopWaveform();
    }
}

// Update the StartWaveform function to reset waveform variables
void Output::StartWaveform() {
    _waveActive = true;
    switch (_waveformType) {
    case WaveformType::Square:
        SetPulse(true);
        break;
    case WaveformType::Triangle:
    case WaveformType::Sawtooth:
    case WaveformType::Sine:
        if (!_externalClock) {
            _waveValue = 0.0f;
            _triangleWaveStep = 0.0f; // Will be calculated in GenerateTriangleWave()
            _sineWaveAngle = 0.0f;
        }
        break;
    case WaveformType::Random:
    case WaveformType::SmoothRandom:
    default:
        SetPulse(true);
        break;
    }
}

void Output::ResetWaveform() {
    _waveActive = false;
    _waveDirection = true;
    _waveValue = 0.0f;
    _triangleWaveStep = 0.0f;
    _sineWaveAngle = 0.0f;
}

// Function to stop waveform generation
void Output::StopWaveform() {
    switch (_waveformType) {
    case WaveformType::Square:
        SetPulse(false);
        _waveActive = false;
        break;
    case WaveformType::SmoothRandom:
    case WaveformType::Random:
    case WaveformType::Triangle:
    case WaveformType::Sawtooth:
    case WaveformType::Sine:
    default:
        SetPulse(false);
        break;
    }
}

// Implement waveform generation functions
void Output::GenerateTriangleWave(int PPQN) {
    if (_waveActive) {
        // Calculate the total ticks for one full waveform period
        float ticksPerPeriod = (PPQN / _clockDividers[_dividerIndex]);

        // Calculate the step size based on the amplitude range and ticks per period
        _triangleWaveStep = (200.0f) / ticksPerPeriod; // Amplitude range [0,100]

        // Update waveform value based on direction
        _waveValue += _waveDirection ? _triangleWaveStep : -_triangleWaveStep;

        // Reverse direction at peak values
        if (_waveValue >= 100.0f) {
            _waveValue = 100.0f;
            _waveDirection = false;
        } else if (_waveValue <= 0.0f) {
            _waveValue = 0.0f;
            _waveDirection = true;
        }
        _isPulseOn = true;
    }
}

// Function to generate a sine wave synchronized with PPQN
void Output::GenerateSineWave(int PPQN) {
    if (_waveActive) {
        // Calculate the period of the waveform in ticks
        float periodInTicks = (PPQN / _clockDividers[_dividerIndex]);

        // Calculate the angle increment per tick
        float angleIncrement = (2.0f * PI) / periodInTicks;

        // Update the angle for the sine function
        _sineWaveAngle += angleIncrement;

        // Keep the angle within 0 to 2*PI
        if (_sineWaveAngle >= 2.0f * PI) {
            _sineWaveAngle -= 2.0f * PI;
        }
        // Apply phase shift to align the lowest point with pulse start
        float shiftedAngle = _sineWaveAngle + (3.0f * PI / 2.0f);

        // Calculate the sine value scaled to the amplitude range [0, 100]
        _waveValue = (sin(shiftedAngle) * 50.0f) + 50.0f;

        // Set the output level based on the waveform value
        _isPulseOn = true;
    }
}

// Function to generate a sawtooth wave synchronized with PPQN
void Output::GenerateSawtoothWave(int PPQN) {
    if (_waveActive) {
        // Calculate the period of the waveform in ticks
        float periodInTicks = (PPQN / _clockDividers[_dividerIndex]) * 2;

        // Calculate the step size based on the amplitude range and ticks per period
        _triangleWaveStep = (200.0f) / periodInTicks; // Amplitude range [0,100]

        // Update waveform value based on direction
        _waveValue += _triangleWaveStep;

        // Reset the waveform value at peak value
        if (_waveValue >= 100.0f) {
            _waveValue = 0.0f;
        }

        _isPulseOn = true;
    }
}

void Output::GenerateRandomWave(int PPQN) {
    if (_waveActive) {
        // Generate white noise waveform
        _waveValue = random(101); // Random value between 0 and 100
        _isPulseOn = true;
    }
}

void Output::GenerateSmoothRandomWave(int PPQN) {
    if (_waveActive) {
        // Generate smooth random waveform using a low-pass filter approach
        float alpha = 0.1;                                           // Smoothing factor
        float randomValue = random(101);                             // Random value between 0 and 100
        _waveValue = alpha * randomValue + (1 - alpha) * _waveValue; // Low-pass filter
        _isPulseOn = true;
    }
}

// Check if the pulse state has changed
bool Output::HasPulseChanged() {
    bool pulseChanged = (_isPulseOn != _lastPulseState);
    _lastPulseState = _isPulseOn;
    return pulseChanged;
}

// Master stop, stops all outputs but on resume, the outputs will resume to previous state
void Output::ToggleMasterState() {
    _masterState = !_masterState;
    if (!_masterState) {
        _oldState = _state;
        _state = false;
    } else {
        _state = _oldState;
    }
}

// Output Level based on the output type and pulse state
int Output::GetOutputLevel() {
    if (_outputType == OutputType::DigitalOut) {
        return _isPulseOn ? HIGH : LOW;
    } else {
        int adjustedLevel;
        switch (_waveformType) {
        case WaveformType::Square:
            adjustedLevel = _isPulseOn ? (_level + _offset) : _offset;
            adjustedLevel = constrain(adjustedLevel, 0, 100);
            return adjustedLevel * MaxDACValue / 100;
        case WaveformType::Triangle:
        case WaveformType::Sine:
        case WaveformType::Sawtooth:
        case WaveformType::Random:
        case WaveformType::SmoothRandom:
            // Take into account the wave value and the _level and _offset values
            adjustedLevel = _isPulseOn ? constrain((_waveValue * _level) / 100 + _offset, 0, 100) : _offset;
            return adjustedLevel * MaxDACValue / 100;
        default:
            return 0;
        }
    }
}

// Euclidean Rhythm Functions
void Output::SetEuclidean(bool enabled) {
    _euclideanParams.enabled = enabled;
    if (_euclideanParams.enabled) {
        GeneratePattern(_euclideanParams, _euclideanRhythm);
    }
}

// Set the number of steps in the Euclidean rhythm
void Output::SetEuclideanSteps(int steps) {
    _euclideanParams.steps = constrain(steps, 1, MaxEuclideanSteps);
    // Ensure that the number of triggers is less than the number of steps
    if (_euclideanParams.triggers > _euclideanParams.steps) {
        _euclideanParams.triggers = _euclideanParams.steps;
    }
    if (_euclideanParams.enabled) {
        GeneratePattern(_euclideanParams, _euclideanRhythm);
    }
}

// Set the number of triggers in the Euclidean rhythm
void Output::SetEuclideanTriggers(int triggers) {
    _euclideanParams.triggers = constrain(triggers, 1, _euclideanParams.steps);
    if (_euclideanParams.enabled) {
        GeneratePattern(_euclideanParams, _euclideanRhythm);
    }
}

// Set the rotation of the Euclidean rhythm
void Output::SetEuclideanRotation(int rotation) {
    _euclideanParams.rotation = constrain(rotation, 0, _euclideanParams.steps - 1);
    if (_euclideanParams.enabled) {
        GeneratePattern(_euclideanParams, _euclideanRhythm);
    }
}

void Output::SetEuclideanPadding(int pad) {
    _euclideanParams.pad = constrain(pad, 0, MaxEuclideanSteps - _euclideanParams.steps);
    if (_euclideanParams.enabled) {
        GeneratePattern(_euclideanParams, _euclideanRhythm);
    }
}
