#include <Arduino.h>

#include "euclidean.hpp"
#include "utils.hpp"

class Output {

  public:
    // Constructor
    Output(int ID, int type);
    // Pulse State
    void Pulse(int PPQN, unsigned long tickCounter);
    bool GetPulseState() { return _isPulseOn; }
    void SetPulse(bool state) { _isPulseOn = state; }
    void TogglePulse() { _isPulseOn = !_isPulseOn; }
    bool HasPulseChanged();
    void SetExternalClock(bool state) { _externalClock = state; }
    // Output State
    bool GetOutputState() { return _state; }
    void SetOutputState(bool state) { _state = state; }
    void ToggleOutputState() { _state = !_state; }
    void ToggleMasterState();
    // Divider
    int GetDividerIndex() { return _dividerIndex; }
    void SetDivider(int index);
    void IncreaseDivider() { SetDivider(_dividerIndex + 1); }
    void DecreaseDivider() { SetDivider(_dividerIndex - 1); }
    String GetDividerDescription() { return _dividerDescription[_dividerIndex]; }
    int GetDividerAmounts() { return dividerAmount; }
    // Duty Cycle
    int GetDutyCycle() { return _dutyCycle; }
    void SetDutyCycle(int dutyCycle) { _dutyCycle = constrain(dutyCycle, 1, 99); }
    void IncreaseDutyCycle() { SetDutyCycle(_dutyCycle + 1); }
    void DecreaseDutyCycle() { SetDutyCycle(_dutyCycle - 1); }
    String GetDutyCycleDescription() { return String(_dutyCycle) + "%"; }
    // Output Level
    int GetLevel() { return _level; }
    int GetOutputLevel(); // Output Level based on the output type
    String GetLevelDescription() { return String(_level) + "%"; }
    void SetLevel(int level) { _level = constrain(level, 0, 100); }
    void IncreaseLevel() { SetLevel(_level + 1); }
    void DecreaseLevel() { SetLevel(_level - 1); }
    // Swing
    void SetSwingAmount(int swingAmount) { _swingAmountIndex = constrain(swingAmount, 0, 6); }
    int GetSwingAmountIndex() { return _swingAmountIndex; }
    int GetSwingAmounts() { return 7; }
    void IncreaseSwingAmount() { SetSwingAmount(_swingAmountIndex + 1); }
    void DecreaseSwingAmount() { SetSwingAmount(_swingAmountIndex - 1); }
    String GetSwingAmountDescription() { return _swingAmountDescriptions[_swingAmountIndex]; }
    void SetSwingEvery(int swingEvery) { _swingEvery = constrain(swingEvery, 1, 16); }
    int GetSwingEvery() { return _swingEvery; }
    void IncreaseSwingEvery() { SetSwingEvery(_swingEvery + 1); }
    void DecreaseSwingEvery() { SetSwingEvery(_swingEvery - 1); }
    // Pulse Probability
    void SetPulseProbability(int pulseProbability) { _pulseProbability = constrain(pulseProbability, 0, 100); }
    int GetPulseProbability() { return _pulseProbability; }
    String GetPulseProbabilityDescription() { return String(_pulseProbability) + "%"; }
    void IncreasePulseProbability() { SetPulseProbability(_pulseProbability + 1); }
    void DecreasePulseProbability() { SetPulseProbability(_pulseProbability - 1); }
    // Euclidean Rhythm
    void SetEuclidean(bool euclidean);
    void ToggleEuclidean() { SetEuclidean(!_euclideanParams.enabled); }
    bool GetEuclidean() { return _euclideanParams.enabled; }
    int GetRhythmStep(int i) { return _euclideanRhythm[i]; }
    void SetEuclideanSteps(int steps);
    int GetEuclideanSteps() { return _euclideanParams.steps; }
    void IncreaseEuclideanSteps() { SetEuclideanSteps(_euclideanParams.steps + 1); }
    void DecreaseEuclideanSteps() { SetEuclideanSteps(_euclideanParams.steps - 1); }
    void SetEuclideanTriggers(int triggers);
    int GetEuclideanTriggers() { return _euclideanParams.triggers; }
    void IncreaseEuclideanTriggers() { SetEuclideanTriggers(_euclideanParams.triggers + 1); }
    void DecreaseEuclideanTriggers() { SetEuclideanTriggers(_euclideanParams.triggers - 1); }
    void SetEuclideanRotation(int rotation);
    int GetEuclideanRotation() { return _euclideanParams.rotation; }
    void IncreaseEuclideanRotation() { SetEuclideanRotation(_euclideanParams.rotation + 1); }
    void DecreaseEuclideanRotation() { SetEuclideanRotation(_euclideanParams.rotation - 1); }

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
    bool _externalClock = false;  // External clock state
    int _outputType;              // 0 = Digital, 1 = DAC
    int _dividerIndex = 5;        // Default to 1
    int _dutyCycle = 50;          // Default to 50%
    int _level = 100;             // Default to 100%
    bool _isPulseOn = false;      // Pulse state
    bool _lastPulseState = false; // Last pulse state
    bool _state = true;           // Output state
    bool _oldState = true;        // Previous output state (for master stop)
    bool _masterState = true;     // Master output state
    int _pulseProbability = 100;  // % chance of pulse

    // Swing variables
    unsigned int _swingAmountIndex = 0; // Swing amount index
    int _swingEvery = 2;                // Swing every x notes

    // Euclidean rhythm variables
    int _euclideanStepIndex = 0;             // Current step in the pattern
    EuclideanParams _euclideanParams;        // Euclidean rhythm parameters
    int _euclideanRhythm[MaxEuclideanSteps]; // Euclidean rhythm pattern
};

// Constructor
Output::Output(int ID, int type) {
    _ID = ID;
    _outputType = type;
    GeneratePattern(_euclideanParams, _euclideanRhythm);
}

// Pulse function
void Output::Pulse(int PPQN, unsigned long tickCounter) {
    // Calculate the tick counter with swing applied
    unsigned long tickCounterSwing = tickCounter;
    if (int(tickCounter / (PPQN / _clockDividers[_dividerIndex])) % _swingEvery == 0) {
        tickCounterSwing = tickCounter - _swingAmounts[_swingAmountIndex];
    }

    // If not stopped, generate the pulse
    if (_state) {
        // Calculate the pulse duration based on the duty cycle
        int _pulseDuration = int(PPQN / _clockDividers[_dividerIndex] * (_dutyCycle / 100.0));
        // If the tick counter is a multiple of the clock divider, generate a pulse
        if (tickCounterSwing % int(PPQN / _clockDividers[_dividerIndex]) == 0 || (tickCounter == 0)) {
            if (!_euclideanParams.enabled) {
                // If not using Euclidean rhythm, generate a pulse based on the pulse probability
                if (random(100) < _pulseProbability) {
                    SetPulse(true);
                }
            } else {
                // If using Euclidean rhythm, check if the current step is active
                if (_euclideanRhythm[_euclideanStepIndex] == 1) {
                    SetPulse(true);
                }
                _euclideanStepIndex++;
                // Restart the Euclidean rhythm if it reaches the end
                if (_euclideanStepIndex >= _euclideanParams.steps) {
                    _euclideanStepIndex = 0;
                }
            }
            // If the tick counter is not a multiple of the clock divider, turn off the pulse
        } else if (int(tickCounter % int(PPQN / _clockDividers[_dividerIndex])) >= _pulseDuration) {
            SetPulse(false);
        }
    } else {
        SetPulse(false);
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

// Output Level based on the output type
int Output::GetOutputLevel() {
    if (_outputType == 0) {
        if (_level == 0) {
            return LOW;
        } else {
            return HIGH;
        }
    } else {
        return _level * MaxDACValue / 100;
    }
}

// Set the divider index
void Output::SetDivider(int index) {
    // If using external clock, don't allow the divider to be set below x1 (only clock multiplication)
    if (_externalClock) {
        _dividerIndex = constrain(index, 5, dividerAmount - 1);
    } else {
        _dividerIndex = constrain(index, 0, dividerAmount - 1);
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
    _euclideanParams.rotation = rotation;
    if (_euclideanParams.enabled) {
        GeneratePattern(_euclideanParams, _euclideanRhythm);
    }
}
