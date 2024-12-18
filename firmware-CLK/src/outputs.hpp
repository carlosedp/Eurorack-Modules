#include <Arduino.h>

#include "definitions.hpp"
#include "euclidean.hpp"

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
    Parabolic,
    Sawtooth,
    ExpEnvelope,
    LogEnvelope,
    Random,
    SmoothRandom,
    SampleHold
};
String WaveformTypeDescriptions[] = {"Square", "Triangle", "Sine", "Parabolic", "Sawtooth", "ExpEnvelope", "LogEnvelope", "Random", "SmoothRdn", "S&H"};
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
    void SetMasterState(bool state);

    // Divider
    int GetDividerIndex() { return _dividerIndex; }
    void SetDivider(int index) { _dividerIndex = constrain(index, 0, _dividerAmount - 1); }
    String GetDividerDescription() { return _dividerDescription[_dividerIndex]; }
    int GetDividerAmounts() { return _dividerAmount; }

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
    int GetSwingAmounts() { return _swingAmount; }
    String GetSwingAmountDescription() { return _swingAmountDescriptions[_swingAmountIndex]; }
    void SetSwingEvery(int swingEvery) { _swingEvery = constrain(swingEvery, 1, _swingEveryAmount); }
    int GetSwingEvery() { return _swingEvery; }
    int GetSwingEveryAmounts() { return _swingEveryAmount; }

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
    const float MaxWaveValue = 255.0;
    static int const _dividerAmount = 18;
    float _clockDividers[_dividerAmount] = {0.0078125, 0.015625, 0.03125, 0.0625, 0.125, 0.25, 0.3333333333, 0.5, 0.6666666667, 1.0, 1.5, 2.0, 3.0, 4.0, 8.0, 16.0, 24.0, 32.0};
    String _dividerDescription[_dividerAmount] = {"/128", "/64", "/32", "/16", "/8", "/4", "/3", "/2", "/1.5", "x1", "x1.5", "x2", "x3", "x4", "x8", "x16", "x24", "x32"};
    static int const MaxEuclideanSteps = 64;

    // The shuffle of the TR-909 delays each even-numbered 1/16th by 2/96 of a beat for shuffle setting 1,
    // 4/96 for 2, 6/96 for 3, 8/96 for 4, 10/96 for 5 and 12/96 for 6.
    static int const _swingAmount = 7;
    float _swingAmounts[_swingAmount] = {0, 2, 4, 6, 8, 10, 12};
    String _swingAmountDescriptions[_swingAmount] = {"0", "2/96", "4/96", "6/96", "8/96", "10/96", "12/96"};

    // Variables
    int _ID;
    bool _externalClock = false;             // External clock state
    OutputType _outputType;                  // 0 = Digital, 1 = DAC
    int _dividerIndex = 9;                   // Default to 1
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
    unsigned long _inactiveTickCounter = 0;
    unsigned long _randomTickCounter = 0;
    unsigned long _envTickCounter = 0; // Logarithmic envelope ticks

    // Swing variables
    int _swingEveryAmount = 16;         // Max swing every value
    int _swingEvery = 2;                // Swing every x notes
    unsigned int _swingAmountIndex = 0; // Swing amount index

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
    void GenerateParabolicWave(int);
    void GenerateSawtoothWave(int);
    void GenerateRandomWave(int);
    void GenerateSmoothRandomWave(int);
    void GenerateExpEnvelope(int);
    void GenerateLogEnvelope(int);
    void GenerateSampleHold(int);
};

// Constructor
Output::Output(int ID, OutputType type) {
    _ID = ID;
    _outputType = type;
    GeneratePattern(_euclideanParams, _euclideanRhythm);
}

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
            tickCounterSwing = globalTick - (_swingAmounts[_swingAmountIndex] * PPQN / 96); // Since our swing is in 96th notes
        }

        // Calculate the clock divider for external clock
        int clockDividerExternal = 1 / _clockDividers[_dividerIndex];

        // Calculate the pulse duration (in ticks) based on the duty cycle
        unsigned int _pulseDuration = int(periodTicks * (_dutyCycle / 100.0));
        unsigned int _externalPulseDuration = int(clockDividerExternal * (_dutyCycle / 100.0));

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
            } else if (_internalPulseCounter % clockDividerExternal == _externalPulseDuration) {
                StopWaveform();
            }
        } else {
            // Handle internal clock timing
            if ((tickCounterSwing - phaseOffsetTicks) % int(periodTicks) == 0 || (globalTick == 0)) {
                generatePulse();
            } else if ((tickCounterSwing - phaseOffsetTicks) % int(periodTicks) == _pulseDuration) {
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
        case WaveformType::Parabolic:
            GenerateParabolicWave(PPQN);
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
        case WaveformType::ExpEnvelope:
            GenerateExpEnvelope(PPQN);
            break;
        case WaveformType::LogEnvelope:
            GenerateLogEnvelope(PPQN);
            break;
        case WaveformType::SampleHold:
            GenerateSampleHold(PPQN);
            break;
        default:
            // For square wave or other types
            break;
        }
    } else {
        StopWaveform();
    }
}

// Start the waveform generation
void Output::StartWaveform() {
    _waveActive = true;
    switch (_waveformType) {
    case WaveformType::Square:
        SetPulse(true);
        break;
    case WaveformType::Triangle:
    case WaveformType::Sawtooth:
    case WaveformType::Sine:
    case WaveformType::Parabolic:
        if (!_externalClock) {
            _waveValue = 0.0f;
            _triangleWaveStep = 0.0f;
            _sineWaveAngle = 0.0f;
        }
        break;
    case WaveformType::ExpEnvelope:
    case WaveformType::LogEnvelope:
        _waveValue = MaxWaveValue; // Start at maximum value for envelopes
        _envTickCounter = 0;
        break;
    case WaveformType::Random:
    case WaveformType::SmoothRandom:
    case WaveformType::SampleHold:
        _randomTickCounter = 0;
    default:
        SetPulse(true);
        break;
    }
}

// Reset the waveform values
void Output::ResetWaveform() {
    switch (_waveformType) {
    case WaveformType::ExpEnvelope:
    case WaveformType::LogEnvelope:
        break;
    default:
        _waveActive = false;
        _waveDirection = true;
        _waveValue = 0.0f;
        _triangleWaveStep = 0.0f;
        _sineWaveAngle = 0.0f;
    }
}

// Function to stop waveform generation
void Output::StopWaveform() {
    switch (_waveformType) {
    case WaveformType::Square:
        SetPulse(false);
        _waveActive = false;
        break;
    case WaveformType::ExpEnvelope:
    case WaveformType::LogEnvelope:
        break;
    case WaveformType::SmoothRandom:
    case WaveformType::Random:
    case WaveformType::Triangle:
    case WaveformType::Sawtooth:
    case WaveformType::Sine:
    case WaveformType::Parabolic:
    case WaveformType::SampleHold:
    default:
        SetPulse(false);
        break;
    }
}

// Generate a triangle wave
void Output::GenerateTriangleWave(int PPQN) {
    if (_waveActive) {
        float periodTicks = PPQN / _clockDividers[_dividerIndex];
        float risingTicks = periodTicks * (_dutyCycle / 100.0f);
        float fallingTicks = periodTicks - risingTicks;

        // Calculate step sizes
        float risingStep = MaxWaveValue / risingTicks;
        float fallingStep = MaxWaveValue / fallingTicks;

        // Update waveform value
        if (_waveDirection) {
            _waveValue += risingStep;
            if (_waveValue >= MaxWaveValue) {
                _waveValue = MaxWaveValue;
                _waveDirection = false;
            }
        } else {
            _waveValue -= fallingStep;
            if (_waveValue <= 0.0f) {
                _waveValue = 0.0f;
                _waveDirection = true;
            }
        }
        _isPulseOn = true;
    }
}

// Generate a sine wave
void Output::GenerateSineWave(int PPQN) {
    if (_waveActive) {
        // Calculate the period of the waveform in ticks
        float periodInTicks = PPQN / _clockDividers[_dividerIndex];

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

        // Calculate the sine value scaled to the amplitude range
        // Adjust the sine value based on the duty cycle
        float sineValue = sin(shiftedAngle);
        if (sineValue > 0) {
            sineValue = pow(sineValue, _dutyCycle / 50.0f);
        } else {
            sineValue = -pow(-sineValue, _dutyCycle / 50.0f);
        }
        _waveValue = (sineValue * MaxWaveValue / 2) + MaxWaveValue / 2;

        _isPulseOn = true;
    }
}

// Generate a parabolic wave
void Output::GenerateParabolicWave(int PPQN) {
    if (_waveActive) {
        float periodInTicks = PPQN / _clockDividers[_dividerIndex];
        float activeTicks = periodInTicks * (_dutyCycle / 100.0f);
        float angleIncrement = (PI) / activeTicks; // Half sine wave for duty cycle

        _sineWaveAngle += angleIncrement;
        if (_sineWaveAngle >= PI) {
            _sineWaveAngle = 0.0f;
            // Inactive period
            _waveActive = false;
            _isPulseOn = false;
            _inactiveTickCounter = periodInTicks - activeTicks;
        }

        // Calculate sine value
        _waveValue = sin(_sineWaveAngle) * MaxWaveValue;
        _isPulseOn = true;
    }
}

// Generate a sawtooth wave
void Output::GenerateSawtoothWave(int PPQN) {
    if (_waveActive) {
        float periodTicks = PPQN / _clockDividers[_dividerIndex];
        float activeTicks = periodTicks * (_dutyCycle / 100.0f);
        float inactiveTicks = periodTicks - activeTicks;

        // Calculate step size
        float step = MaxWaveValue / activeTicks;

        // Update waveform value
        _waveValue += step;
        if (_waveValue >= MaxWaveValue) {
            _waveValue = 0.0f;
            // Adjust for inactive period
            _waveActive = false;
            _inactiveTickCounter = inactiveTicks;
        }
        _isPulseOn = true;
    } else {
        // Handle inactive period
        _inactiveTickCounter--;
        if (_inactiveTickCounter <= 0) {
            _waveActive = true;
        }
        _isPulseOn = false;
    }
}

// Generate random values
void Output::GenerateRandomWave(int PPQN) {
    if (_waveActive) {
        // Generate white noise waveform
        _waveValue = random(MaxWaveValue + 1); // Random value
        _isPulseOn = true;
        _randomTickCounter++;
    }
}

// Generate smooth random waveform
void Output::GenerateSmoothRandomWave(int PPQN) {
    if (_waveActive) {
        // Generate smooth random waveform with smooth peaks and valleys
        static float phase = 0.0f;
        static float frequency = 0.3f;             // Adjust frequency for smoothness
        static float amplitude = MaxWaveValue / 2; // Amplitude for wave value range
        static float lastValue = MaxWaveValue / 2; // Last generated value
        static float smoothValue = 50.0f;          // Smoothed value

        // Increment phase
        phase += frequency;

        // Generate smooth random value using a random walk
        float randomStep = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0f; // Random step between -1 and 1
        lastValue += randomStep * amplitude * frequency;                          // Adjust step size by amplitude and frequency
        lastValue = fmin(fmax(lastValue, 0.0f), MaxWaveValue);                    // Clamp value

        // Apply a low-pass filter to smooth out the waveform
        float alpha = 0.01f; // Smoothing factor (0 < alpha < 1)
        smoothValue = alpha * lastValue + (1.0f - alpha) * smoothValue;

        _waveValue = smoothValue;

        _isPulseOn = true;
    }
}

// Generate an exponential envelope waveform
void Output::GenerateExpEnvelope(int PPQN) {
    if (_waveActive) {
        float periodTicks = PPQN / _clockDividers[_dividerIndex];
        float decayTicks = periodTicks * (_dutyCycle / 100.0f);

        if (_envTickCounter >= decayTicks) {
            _waveValue = 0.0f;
            _waveActive = false;
            _envTickCounter = 0; // Reset for the next pulse
            return;
        }

        // Calculate decay factor for the exponential decay
        float k = 6.90776f / decayTicks; // ln(100) ≈ 4.60517, total decay over decayTicks
        _waveValue = MaxWaveValue * exp(-k * _envTickCounter);

        _envTickCounter++;
        _isPulseOn = true;
    }
}

// Generate a logarithm envelope waveform
void Output::GenerateLogEnvelope(int PPQN) {
    if (_waveActive) {
        float periodTicks = PPQN / _clockDividers[_dividerIndex];
        float decayTicks = periodTicks * (_dutyCycle / 100.0f);

        if (_envTickCounter >= decayTicks) {
            _waveValue = 0.0f;
            _waveActive = false;
            _envTickCounter = 0; // Reset for the next pulse
            return;
        }

        // Calculate decay factor to span the entire pulse duration
        float decayFactor = log10(decayTicks - _envTickCounter + 1) / log10(decayTicks + 1);

        // Update waveform value
        _waveValue = decayFactor * MaxWaveValue; // Scale to 0-100%

        _envTickCounter++;
        _isPulseOn = true;
    }
}

// Generate a Sample and Hold waveform where on each pulse, a random value is generated
void Output::GenerateSampleHold(int PPQN) {
    if (_waveActive) {
        // Generate a random value at the start of each pulse
        if (_randomTickCounter == 0) {
            _waveValue = random(MaxWaveValue + 1);
        }
        _isPulseOn = true;
        _randomTickCounter++;
    }
}

// Check if the pulse state has changed
bool Output::HasPulseChanged() {
    bool pulseChanged = (_isPulseOn != _lastPulseState);
    _lastPulseState = _isPulseOn;
    return pulseChanged;
}

void Output::SetMasterState(bool state) {
    if (_masterState != state) {
        _masterState = state;
        if (!_masterState) {
            _oldState = _state;
            _state = false;
        } else {
            _state = _oldState;
        }
    }
}

// Master stop, stops all outputs but on resume, the outputs will resume to previous state
void Output::ToggleMasterState() {
    SetMasterState(!_masterState);
}

// Output Level based on the output type and pulse state
int Output::GetOutputLevel() {
    float adjustedLevel;
    if (_outputType == OutputType::DigitalOut) {
        return _isPulseOn ? HIGH : LOW;
    } else {
        if (_waveformType == WaveformType::Square) {
            adjustedLevel = _isPulseOn ? (MaxWaveValue * (_level / 100.0)) + ((_offset / 100.0) * MaxWaveValue) : _offset;
            adjustedLevel = constrain(adjustedLevel, 0, MaxWaveValue);
        } else {
            // Take into account the wave value and the _level and _offset values
            adjustedLevel = _waveValue * (_level / 100.0) + (_offset / 100.0) * MaxWaveValue;
            adjustedLevel = constrain(adjustedLevel, 0, MaxWaveValue);
            adjustedLevel = _isPulseOn ? adjustedLevel : _offset;
        }
        return adjustedLevel * MaxDACValue / MaxWaveValue;
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
    if (_euclideanParams.pad > MaxEuclideanSteps - _euclideanParams.steps) {
        _euclideanParams.pad = MaxEuclideanSteps - _euclideanParams.steps;
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
