#include <Arduino.h>
#include <TimerTCC0.h>
// Rotary encoder setting
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// Load local libraries
#include "boardIO.cpp"
#include "loadsave.cpp"
#include "outputs.hpp"
#include "pinouts.hpp"
#include "splash.hpp"
#include "utils.hpp"
#include "version.hpp"

// Define the amount of clock outputs
#define NUM_OUTPUTS 4

#define PPQN 96

#define OLED_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Assign ADC calibration value for each channel based on the actual measurement
float ADCalibration[2] = {0.99728, 0.99728};

// OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Rotary encoder object
Encoder encoder(ENC_PIN_1, ENC_PIN_2); // rotary encoder library setting
float oldPosition = -999;              // rotary encoder library setting
float newPosition = -999;              // rotary encoder library setting

// Output objects
Output outputs[NUM_OUTPUTS] = {
    Output(1, 0),
    Output(2, 0),
    Output(3, 1),
    Output(4, 1)};

// ---- Global variables ----

// ADC input variables
float channelADC[2], oldChannelADC[2];

// BPM and clock settings
unsigned int BPM = 120;
unsigned int lastInternalBPM = 120;
unsigned int const minBPM = 10;
unsigned int const maxBPM = 300;

// Play/Stop state
bool masterStop = false; // New variable to track play/stop state

// Global tick counter
unsigned long tickCounter = 0;

// External clock variables
volatile unsigned long clockInterval = 0;
volatile unsigned long lastClockInterruptTime = 0;
volatile bool usingExternalClock = false;

static int const dividerAmount = 5;
int externalClockDividers[dividerAmount] = {1, 2, 4, 8, 16};
String externalDividerDescription[dividerAmount] = {"x1", "/2 ", "/4", "/8", "/16"};
int externalDividerIndex = 0;
unsigned long externalTickCounter = 0;

// Menu variables
int menuItems = 39;
int menuItem = 3;
bool switchState = 1;
bool oldSwitchState = 0;
int menuMode = 0;            // Menu mode for parameter editing
bool displayRefresh = 1;     // Display refresh flag
bool unsavedChanges = false; // Unsaved changes flag
int euclideanOutput = 0;     // Euclidean rhythm output index

// Function prototypes
void UpdateBPM(unsigned int);
void SetTapTempo();
void ToggleMasterStop();
void HandleEncoderClick();
void HandleEncoderPosition();
void EncoderSpeedFactor(unsigned long timeDiff);
void HandleDisplay();
void HandleExternalClock();
void HandleCVInputs();
void HandleOutputs();
void ClockPulse(int);
void ClockPulseInternal();
void InitializeTimer();
void UpdateParameters(LoadSaveParams);

// ----------------------------------------------

// Handle encoder button click
void HandleEncoderClick() {
    oldSwitchState = switchState;
    switchState = digitalRead(ENCODER_SW);
    if (switchState == 1 && oldSwitchState == 0) {
        displayRefresh = 1;
        if (menuMode == 0) {
            switch (menuItem) {
            case 1: // Set BPM
                menuMode = 1;
                break;
            case 2: // Toggle stopped state
                ToggleMasterStop();
                break;
            case 3: // Set div1
                menuMode = 3;
                break;
            case 4: // Set div2
                menuMode = 4;
                break;
            case 5: // Set div3
                menuMode = 5;
                break;
            case 6: // Set div4
                menuMode = 6;
                break;
            case 7: // External clock divider
                menuMode = 7;
                break;
            case 8:
                outputs[0].ToggleOutputState();
                unsavedChanges = true;
                break;
            case 9:
                outputs[1].ToggleOutputState();
                unsavedChanges = true;
                break;
            case 10:
                outputs[2].ToggleOutputState();
                unsavedChanges = true;
                break;
            case 11:
                outputs[3].ToggleOutputState();
                unsavedChanges = true;
                break;
            case 12: // Set swing amount for output 1
                menuMode = 12;
                break;
            case 13: // Set swing every for output 1
                menuMode = 13;
                break;
            case 14: // Set swing amount for output 2
                menuMode = 14;
                break;
            case 15: // Set swing every for output 2
                menuMode = 15;
                break;
            case 16: // Set swing amount for output 3
                menuMode = 16;
                break;
            case 17: // Set swing every for output 3
                menuMode = 17;
                break;
            case 18: // Set swing amount for output 4
                menuMode = 18;
                break;
            case 19: // Set swing every for output 4
                menuMode = 19;
                break;
            case 20: // Set pulse probability for outputs
                menuMode = 20;
                break;
            case 21:
                menuMode = 21;
                break;
            case 22:
                menuMode = 22;
                break;
            case 23:
                menuMode = 23;
                break;
            case 24: // Set phase shift for outputs
                menuMode = 24;
                break;
            case 25:
                menuMode = 25;
                break;
            case 26:
                menuMode = 26;
                break;
            case 27:
                menuMode = 27;
                break;
            case 28: // Select Euclidean rhythm output to edit
                menuMode = 24;
                break;
            case 29:
                outputs[euclideanOutput].ToggleEuclidean();
                unsavedChanges = true;
                break;
            case 30: // Set Euclidean rhythm step length
                menuMode = 30;
                break;
            case 31: // Set Euclidean rhythm number of triggers
                menuMode = 31;
                break;
            case 32: // Set Euclidean rhythm rotation
                menuMode = 32;
                break;
            case 33: // Set euclidean padding
                menuMode = 33;
                break;
            case 34: // Set duty cycle
                menuMode = 33;
                break;
            case 35: // Level control for output 3
                menuMode = 34;
                break;
            case 36: // Level control for output 4
                menuMode = 35;
                break;
            case 37:
                SetTapTempo();
                break; // Tap tempo
            case 38: { // Save settings
                LoadSaveParams p;
                p.BPM = BPM;
                p.externalClockDivIdx = externalDividerIndex;
                for (int i = 0; i < NUM_OUTPUTS; i++) {
                    p.divIdx[i] = outputs[i].GetDividerIndex();
                    p.dutyCycle[i] = outputs[i].GetDutyCycle();
                    p.outputState[i] = outputs[i].GetOutputState();
                    p.level[i] = outputs[i].GetLevel();
                    p.swingIdx[i] = outputs[i].GetSwingAmountIndex();
                    p.swingEvery[i] = outputs[i].GetSwingEvery();
                    p.pulseProbability[i] = outputs[i].GetPulseProbability();
                    p.euclideanEnabled[i] = outputs[i].GetEuclidean();
                    p.euclideanSteps[i] = outputs[i].GetEuclideanSteps();
                    p.euclideanTriggers[i] = outputs[i].GetEuclideanTriggers();
                    p.euclideanRotations[i] = outputs[i].GetEuclideanRotation();
                    p.euclideanPadding[i] = outputs[i].GetEuclideanPadding();
                    p.phaseShift[i] = outputs[i].GetPhase();
                }
                Save(p, 0);
                unsavedChanges = false;
                display.clearDisplay(); // clear display
                display.setTextSize(2);
                display.setCursor(SCREEN_WIDTH / 2 - 30, SCREEN_HEIGHT / 2 - 16);
                display.print("SAVED");
                display.display();
                unsigned long saveMessageStartTime = millis();
                while (millis() - saveMessageStartTime < 1000) {
                    HandleEncoderClick();
                    HandleEncoderPosition();
                    HandleCVInputs();
                    HandleExternalClock();
                    HandleOutputs();
                }
                break;
            }
            case 39: // Load default settings
                LoadSaveParams p = LoadDefaultParams();
                UpdateParameters(p);
                unsavedChanges = false;
                display.clearDisplay(); // clear display
                display.setTextSize(2);
                display.setCursor(SCREEN_WIDTH / 2 - 30, SCREEN_HEIGHT / 2 - 16);
                display.print("LOADED");
                display.display();
                unsigned long loadMessageStartTime = millis();
                while (millis() - loadMessageStartTime < 1000) {
                    HandleEncoderClick();
                    HandleEncoderPosition();
                    HandleCVInputs();
                    HandleExternalClock();
                    HandleOutputs();
                }
                break;
            }
        } else {
            menuMode = 0;
        }
    }
}

// Calculate the speed of the encoder rotation
float speedFactor;
unsigned long lastEncoderTime = 0;
void UpdateSpeedFactor() {
    unsigned long currentEncoderTime = millis();
    unsigned long timeDiff = currentEncoderTime - lastEncoderTime;
    lastEncoderTime = currentEncoderTime;

    if (timeDiff < 50) {
        speedFactor = 8.0; // Fast rotation
    } else if (timeDiff < 100) {
        speedFactor = 4.0; // Medium rotation
    } else if (timeDiff < 200) {
        speedFactor = 2.0; // Slow rotation
    } else {
        speedFactor = 1.0; // Normal rotation
    }
}

void HandleEncoderPosition() {
    newPosition = encoder.read();

    if ((newPosition - 3) / 4 > oldPosition / 4) { // Decrease, turned counter-clockwise
        UpdateSpeedFactor();
        oldPosition = newPosition;
        displayRefresh = 1;
        switch (menuMode) {
        case 0:
            menuItem = (menuItem - 1 < 1) ? menuItems : menuItem - 1;
            break;
        case 1: // Set BPM
            UpdateBPM(BPM - speedFactor);
            unsavedChanges = true;
            break;
        case 3:
        case 4:
        case 5:
        case 6: // Set div1, div2, div3, div4
            for (int i = 0; i < speedFactor; i++) {
                outputs[menuMode - 3].DecreaseDivider();
            }
            unsavedChanges = true;
            break;
        case 7: // External clock divider
            externalDividerIndex = constrain(externalDividerIndex - speedFactor, 0, dividerAmount - 1);
            unsavedChanges = true;
            break;
        case 12:
            for (int i = 0; i < speedFactor; i++) {
                outputs[0].DecreaseSwingAmount();
            }
            unsavedChanges = true;
            break;
        case 13:
            for (int i = 0; i < speedFactor; i++) {
                outputs[0].DecreaseSwingEvery();
            }
            unsavedChanges = true;
            break;
        case 14:
            for (int i = 0; i < speedFactor; i++) {
                outputs[1].DecreaseSwingAmount();
            }
            unsavedChanges = true;
            break;
        case 15:
            for (int i = 0; i < speedFactor; i++) {
                outputs[1].DecreaseSwingEvery();
            }
            unsavedChanges = true;
            break;
        case 16:
            for (int i = 0; i < speedFactor; i++) {
                outputs[2].DecreaseSwingAmount();
            }
            unsavedChanges = true;
            break;
        case 17:
            for (int i = 0; i < speedFactor; i++) {
                outputs[2].DecreaseSwingEvery();
            }
            unsavedChanges = true;
            break;
        case 18:
            for (int i = 0; i < speedFactor; i++) {
                outputs[3].DecreaseSwingAmount();
            }
            unsavedChanges = true;
            break;
        case 19:
            for (int i = 0; i < speedFactor; i++) {
                outputs[3].DecreaseSwingEvery();
            }
            unsavedChanges = true;
            break;
        case 20:
        case 21:
        case 22:
        case 23: // Set Pulse Probability for outputs
            for (int i = 0; i < speedFactor; i++) {
                outputs[menuMode - 20].DecreasePulseProbability();
            }
            unsavedChanges = true;
            break;
        case 24:
        case 25:
        case 26:
        case 27: // Set phase shift for outputs
            for (int i = 0; i < speedFactor; i++) {
                outputs[menuMode - 24].DecreasePhase();
            }
            unsavedChanges = true;
            break;
        case 28: // Set euclidean output to edit
            euclideanOutput = (euclideanOutput - speedFactor < 0) ? NUM_OUTPUTS - 1 : euclideanOutput - speedFactor;
            unsavedChanges = true;
            break;
        case 30: // Set Euclidean rhythm step length
            for (int i = 0; i < speedFactor; i++) {
                outputs[euclideanOutput].DecreaseEuclideanSteps();
            }
            unsavedChanges = true;
            break;
        case 31: // Set Euclidean rhythm number of triggers
            for (int i = 0; i < speedFactor; i++) {
                outputs[euclideanOutput].DecreaseEuclideanTriggers();
            }
            unsavedChanges = true;
            break;
        case 32: // Set Euclidean rhythm rotation
            for (int i = 0; i < speedFactor; i++) {
                outputs[euclideanOutput].DecreaseEuclideanRotation();
            }
            unsavedChanges = true;
            break;
        case 33: // Set Euclidean padding
            for (int i = 0; i < speedFactor; i++) {
                outputs[euclideanOutput].DecreaseEuclideanPadding();
            }
            unsavedChanges = true;
            break;
        case 34: // Set duty cycle
            for (int i = 0; i < NUM_OUTPUTS; i++) {
                for (int j = 0; j < speedFactor; j++) {
                    outputs[i].DecreaseDutyCycle();
                }
            }
            unsavedChanges = true;
            break;
        case 35: // Set level for output 3
            for (int i = 0; i < speedFactor; i++) {
                outputs[2].DecreaseLevel();
            }
            unsavedChanges = true;
            break;
        case 36: // Set level for output 4
            for (int i = 0; i < speedFactor; i++) {
                outputs[3].DecreaseLevel();
            }
            unsavedChanges = true;
            break;
        }
    } else if ((newPosition + 3) / 4 < oldPosition / 4) { // Increase, turned clockwise
        UpdateSpeedFactor();
        oldPosition = newPosition;
        displayRefresh = 1;
        switch (menuMode) {
        case 0:
            menuItem = (menuItem + 1 > menuItems) ? 1 : menuItem + 1;
            break;
        case 1: // Set BPM
            UpdateBPM(BPM + speedFactor);
            unsavedChanges = true;
            break;
        case 3:
        case 4:
        case 5:
        case 6: // Set div1, div2, div3, div4
            for (int i = 0; i < speedFactor; i++) {
                outputs[menuMode - 3].IncreaseDivider();
            }
            unsavedChanges = true;
            break;
        case 7: // External clock divider
            externalDividerIndex = constrain(externalDividerIndex + speedFactor, 0, dividerAmount - 1);
            unsavedChanges = true;
            break;
        case 12:
            for (int i = 0; i < speedFactor; i++) {
                outputs[0].IncreaseSwingAmount();
            }
            unsavedChanges = true;
            break;
        case 13:
            for (int i = 0; i < speedFactor; i++) {
                outputs[0].IncreaseSwingEvery();
            }
            unsavedChanges = true;
            break;
        case 14:
            for (int i = 0; i < speedFactor; i++) {
                outputs[1].IncreaseSwingAmount();
            }
            unsavedChanges = true;
            break;
        case 15:
            for (int i = 0; i < speedFactor; i++) {
                outputs[1].IncreaseSwingEvery();
            }
            unsavedChanges = true;
            break;
        case 16:
            for (int i = 0; i < speedFactor; i++) {
                outputs[2].IncreaseSwingAmount();
            }
            unsavedChanges = true;
            break;
        case 17:
            for (int i = 0; i < speedFactor; i++) {
                outputs[2].IncreaseSwingEvery();
            }
            unsavedChanges = true;
            break;
        case 18:
            for (int i = 0; i < speedFactor; i++) {
                outputs[3].IncreaseSwingAmount();
            }
            unsavedChanges = true;
            break;
        case 19:
            for (int i = 0; i < speedFactor; i++) {
                outputs[3].IncreaseSwingEvery();
            }
            unsavedChanges = true;
            break;
        case 20:
        case 21:
        case 22:
        case 23: // Set Pulse Probability for outputs
            for (int i = 0; i < speedFactor; i++) {
                outputs[menuMode - 20].IncreasePulseProbability();
            }
            unsavedChanges = true;
            break;
        case 24:
        case 25:
        case 26:
        case 27: // Set phase shift for outputs
            for (int i = 0; i < speedFactor; i++) {
                outputs[menuMode - 24].IncreasePhase();
            }
            unsavedChanges = true;
            break;
        case 28: // Set euclidean output to edit
            euclideanOutput = (euclideanOutput + speedFactor > NUM_OUTPUTS - 1) ? 0 : euclideanOutput + speedFactor;
            unsavedChanges = true;
            break;
        case 30: // Set Euclidean rhythm step length
            for (int i = 0; i < speedFactor; i++) {
                outputs[euclideanOutput].IncreaseEuclideanSteps();
            }
            unsavedChanges = true;
            break;
        case 31: // Set Euclidean rhythm number of triggers
            for (int i = 0; i < speedFactor; i++) {
                outputs[euclideanOutput].IncreaseEuclideanTriggers();
            }
            unsavedChanges = true;
            break;
        case 32: // Set Euclidean rhythm rotation
            for (int i = 0; i < speedFactor; i++) {
                outputs[euclideanOutput].IncreaseEuclideanRotation();
            }
            unsavedChanges = true;
            break;
        case 33: // Set Euclidean padding
            for (int i = 0; i < speedFactor; i++) {
                outputs[euclideanOutput].IncreaseEuclideanPadding();
            }
            unsavedChanges = true;
            break;
        case 34: // Set duty cycle
            for (int i = 0; i < NUM_OUTPUTS; i++) {
                for (int j = 0; j < speedFactor; j++) {
                    outputs[i].IncreaseDutyCycle();
                }
            }
            unsavedChanges = true;
            break;
        case 35: // Set level for output 3
            for (int i = 0; i < speedFactor; i++) {
                outputs[2].IncreaseLevel();
            }
            unsavedChanges = true;
            break;
        case 36: // Set level for output 4
            for (int i = 0; i < speedFactor; i++) {
                outputs[3].IncreaseLevel();
            }
            unsavedChanges = true;
            break;
        }
    }
}

// Redraw the display and show unsaved changes indicator
void RedrawDisplay() {
    // If there are unsaved changes, display an asterisk at the top right corner
    if (unsavedChanges) {
        display.setCursor(120, 0);
        display.print("*");
    }
    display.display();
    displayRefresh = 0;
}

// Handle display drawing
void HandleDisplay() {
    if (displayRefresh == 1) {
        display.clearDisplay();
        int menuIdx = 0;
        int menuItems = 0;

        // Draw the menu
        if (menuItem == 1 || menuItem == 2) {
            display.setCursor(10, 0);
            display.setTextSize(3);
            display.print(BPM);
            display.print("BPM");
            if (usingExternalClock) {
                display.setTextSize(1);
                display.setCursor(120, 24);
                display.print("E");
            }
            // Draw selection triangle
            if (menuMode == 0 && menuItem == 1) {
                display.drawTriangle(2, 6, 2, 14, 6, 10, 1);
            } else if (menuMode == menuItem) {
                display.fillTriangle(2, 6, 2, 14, 6, 10, 1);
            }

            if (menuMode >= 0 && menuMode <= 2) {
                display.setTextSize(2);
                display.setCursor(44, 27);
                if (menuItem == 2) {
                    display.drawLine(43, 42, 88, 42, 1);
                }
                if (masterStop) {
                    display.fillRoundRect(23, 26, 17, 17, 2, 1);
                    display.print("STOP");
                } else {
                    display.fillTriangle(23, 26, 23, 42, 39, 34, 1);
                    display.print("PLAY");
                }
            }

            // Show a box to each output showing if it's enabled
            display.setTextSize(1);
            for (int i = 0; i < NUM_OUTPUTS; i++) {
                display.setCursor((i * 30) + 17, 46);
                display.print(i + 1);
                if (outputs[i].GetOutputState()) {
                    display.fillRect((i * 30) + 16, 56, 8, 8, WHITE);
                } else {
                    display.drawRect((i * 30) + 16, 56, 8, 8, WHITE);
                }
            }
            RedrawDisplay();
            return;
        }

        // Clock dividers menu
        menuIdx = 3;
        menuItems = 5;
        if (menuItem >= menuIdx && menuItem < menuIdx + menuItems) {
            display.setTextSize(1);
            display.setCursor(10, 1);
            display.println("Clock Dividers");
            int yPosition = 20;
            for (int i = 0; i < NUM_OUTPUTS; i++) {
                display.setCursor(10, yPosition);
                display.print("Output " + String(i + 1) + ":");
                display.setCursor(84, yPosition);
                display.print(outputs[i].GetDividerDescription());
                if (menuItem == i + menuIdx) {
                    if (menuMode == 0) {
                        display.drawTriangle(1, 19 + (i * 9), 1, 27 + (i * 9), 5, 23 + (i * 9), 1);
                    } else if (menuMode == i + menuIdx) {
                        display.fillTriangle(1, 19 + (i * 9), 1, 27 + (i * 9), 5, 23 + (i * 9), 1);
                    }
                }
                yPosition += 9;
            }
            // For external clock divider
            display.setCursor(10, yPosition);
            display.print("Ext. Div:");
            display.setCursor(84, yPosition);
            display.print(externalDividerDescription[externalDividerIndex]);
            if (menuItem == 7) {
                if (menuMode == 0) {
                    display.drawTriangle(1, 19 + (NUM_OUTPUTS * 9), 1, 27 + (NUM_OUTPUTS * 9), 5, 23 + (NUM_OUTPUTS * 9), 1);
                } else if (menuMode == 7) {
                    display.fillTriangle(1, 19 + (NUM_OUTPUTS * 9), 1, 27 + (NUM_OUTPUTS * 9), 5, 23 + (NUM_OUTPUTS * 9), 1);
                }
            }

            RedrawDisplay();
            return;
        }

        // Clock outputs state menu
        menuIdx = 8;
        if (menuItem >= menuIdx && menuItem < menuIdx + 4) {
            display.setTextSize(1);
            display.setCursor(10, 1);
            display.println("Output State");
            int yPosition = 20;
            for (int i = 0; i < NUM_OUTPUTS; i++) {
                display.setCursor(10, yPosition);
                display.print("Output " + String(i + 1) + ":");
                display.setCursor(70, yPosition);
                display.print(outputs[i].GetOutputState() ? "On" : "Off");
                if (menuItem == i + menuIdx) {
                    if (menuMode == 0) {
                        display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
                    } else if (menuMode == i + menuIdx) {
                        display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
                    }
                }
                yPosition += 9;
            }
            RedrawDisplay();
            return;
        }

        // Swing amount menu
        menuIdx = 12;
        if (menuItem >= menuIdx && menuItem < menuIdx + 8) {
            display.setTextSize(1);
            display.setCursor(10, 1);
            display.println("Output Swing");
            display.setCursor(64, 20);
            display.println("Amt");
            display.setCursor(94, 20);
            display.println("Every");
            int yPosition = 29;
            for (int i = 0; i < NUM_OUTPUTS; i++) {
                display.setCursor(10, yPosition);
                display.print("Output " + String(i + 1) + ":");
                display.setCursor(70, yPosition);
                display.print(outputs[i].GetSwingAmountDescription());
                display.setCursor(100, yPosition);
                display.print(outputs[i].GetSwingEvery());

                if (menuItem % 2 == 0) {
                    display.fillTriangle(59, 20, 59, 26, 62, 23, 1);
                } else {
                    display.fillTriangle(89, 20, 89, 26, 92, 23, 1);
                }

                if (menuItem - menuIdx == i * 2 || menuItem - menuIdx == i * 2 + 1) {
                    if (menuMode == 0) {
                        display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
                    } else {
                        display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
                    }
                }
                yPosition += 9;
            }
            RedrawDisplay();
            return;
        }

        // Pulse Probability menu
        menuIdx = 20;
        if (menuItem >= menuIdx && menuItem < menuIdx + 4) {
            display.setTextSize(1);
            int yPosition = 0;
            display.setCursor(10, yPosition);
            display.println("Pulse Probability");
            yPosition = 20;
            for (int i = 0; i < NUM_OUTPUTS; i++) {
                display.setCursor(10, yPosition);
                display.print("Output " + String(i + 1) + ":");
                display.setCursor(70, yPosition);
                display.print(outputs[i].GetPulseProbabilityDescription());
                if (menuItem == menuIdx + i) {
                    if (menuMode == 0) {
                        display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
                    } else if (menuMode == menuIdx + i) {
                        display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
                    }
                }
                yPosition += 9;
            }
            RedrawDisplay();
            return;
        }

        // Phase shift menu
        menuIdx = 24;
        if (menuItem >= menuIdx && menuItem < menuIdx + 4) {
            display.setTextSize(1);
            int yPosition = 0;
            display.setCursor(10, yPosition);
            display.println("Phase Shift");
            yPosition = 20;
            for (int i = 0; i < NUM_OUTPUTS; i++) {
                display.setCursor(10, yPosition);
                display.print("Output " + String(i + 1) + ":");
                display.setCursor(70, yPosition);
                display.print(outputs[i].GetPhaseDescription());
                if (menuItem == menuIdx + i) {
                    if (menuMode == 0) {
                        display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
                    } else if (menuMode == menuIdx + i) {
                        display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
                    }
                }
                yPosition += 9;
            }
            RedrawDisplay();
            return;
        }

        // Euclidean rhythm menu
        menuIdx = 28;
        if (menuItem >= menuIdx && menuItem < menuIdx + 6) {
            display.setTextSize(1);
            int yPosition = 0;
            display.setCursor(10, yPosition);
            display.println("Euclidean Rhythm");
            yPosition = 20;
            int xPosition = 64;
            display.setCursor(10, yPosition);
            display.print("Output: ");
            display.setCursor(xPosition, yPosition);
            display.print(String(euclideanOutput + 1));
            if (menuItem == menuIdx && menuMode == 0) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            } else if (menuMode == menuIdx) {
                display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }
            yPosition += 9;
            display.setCursor(10, yPosition);
            display.print("Enabled: ");
            display.setCursor(xPosition, yPosition);
            display.print(String(outputs[euclideanOutput].GetEuclidean() ? "Yes" : "No"));
            if (menuItem == menuIdx + 1 && menuMode == 0) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            } else if (menuMode == menuIdx + 1) {
                display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }
            yPosition += 9;
            display.setCursor(10, yPosition);
            display.print("Steps: ");
            display.setCursor(xPosition, yPosition);
            display.print(String(outputs[euclideanOutput].GetEuclideanSteps()));
            if (menuItem == menuIdx + 2 && menuMode == 0) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            } else if (menuMode == menuIdx + 2) {
                display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }
            yPosition += 9;
            display.setCursor(10, yPosition);
            display.print("Hits: ");
            display.setCursor(xPosition, yPosition);
            display.print(String(outputs[euclideanOutput].GetEuclideanTriggers()));
            if (menuItem == menuIdx + 3 && menuMode == 0) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            } else if (menuMode == menuIdx + 3) {
                display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }
            yPosition += 9;
            display.setCursor(10, yPosition);
            display.print("Rt:");
            display.print(String(outputs[euclideanOutput].GetEuclideanRotation()));
            if (menuItem == menuIdx + 4 && menuMode == 0) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            } else if (menuMode == menuIdx + 4) {
                display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }

            display.setCursor(48, yPosition);
            display.print("Pd:");
            display.print(String(outputs[euclideanOutput].GetEuclideanPadding()));
            if (menuItem == menuIdx + 5 && menuMode == 0) {
                display.drawTriangle(42, yPosition - 1, 42, yPosition + 7, 46, yPosition + 3, 1);
            } else if (menuMode == menuIdx + 5) {
                display.fillTriangle(42, yPosition - 1, 42, yPosition + 7, 46, yPosition + 3, 1);
            }

            // Draw the Euclidean rhythm pattern for the selected output
            if (outputs[euclideanOutput].GetEuclidean()) {
                display.fillTriangle(90, 10, 94, 10, 92, 14, WHITE);
                yPosition = 15;
                int euclideanSteps = outputs[euclideanOutput].GetEuclideanSteps();
                int euclideanPadding = outputs[euclideanOutput].GetEuclideanPadding();
                for (int i = 0; i < euclideanSteps + euclideanPadding && i < 47; i++) {
                    int column = i / 8;
                    int row = i % 8;
                    display.setCursor(90 + (column * 6), yPosition + (row * 6));
                    if (i < euclideanSteps && outputs[euclideanOutput].GetRhythmStep(i)) {
                        display.fillRect(90 + (column * 6), yPosition + (row * 6), 5, 5, WHITE);
                    } else {
                        if (i < euclideanSteps) {
                            display.drawRect(90 + (column * 6), yPosition + (row * 6), 5, 5, WHITE);
                        } else {
                            display.drawRect(90 + (column * 6), yPosition + (row * 6), 5, 5, WHITE);
                            display.writePixel(90 + (column * 6) + 2, yPosition + (row * 6) + 2, WHITE);
                        }
                    }
                }
                if (euclideanSteps + euclideanPadding > 47) {
                    display.fillTriangle(120, 57, 124, 57, 122, 61, WHITE);
                }
            }
            RedrawDisplay();
            return;
        }

        // Duty cycle and level control menu
        menuIdx = 34;
        if (menuItem >= menuIdx && menuItem < menuIdx + 6) {
            display.setTextSize(1);
            int yPosition = 0;
            // Duty Cycle
            display.setCursor(10, yPosition);
            display.println("Duty Cycle:");
            display.setCursor(80, yPosition);
            display.print(outputs[0].GetDutyCycleDescription());
            if (menuItem == menuIdx && menuMode == 0) {
                display.drawTriangle(1, yPosition, 1, yPosition + 8, 5, yPosition + 4, 1);
            } else if (menuMode == menuIdx) {
                display.fillTriangle(1, yPosition, 1, yPosition + 8, 5, yPosition + 4, 1);
            }
            yPosition += 9;
            // Level output 3
            display.setCursor(10, yPosition);
            display.print("Out 3 Lvl:");
            display.setCursor(80, yPosition);
            display.print(outputs[2].GetLevelDescription());
            if (menuItem == menuIdx + 1 && menuMode == 0) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            } else if (menuMode == menuIdx + 1) {
                display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }
            yPosition += 9;
            // Level output 4
            display.setCursor(10, yPosition);
            display.print("Out 4 Lvl:");
            display.setCursor(80, yPosition);
            display.print(outputs[3].GetLevelDescription());
            if (menuItem == menuIdx + 2 && menuMode == 0) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            } else if (menuMode == menuIdx + 2) {
                display.fillTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }
            yPosition += 15;
            // Tap tempo
            display.setCursor(10, yPosition);
            display.print("TAP TEMPO");
            display.print(" (" + String(BPM) + " BPM)");
            if (menuItem == menuIdx + 3) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }
            yPosition += 15;
            // Save
            display.setCursor(10, yPosition);
            display.print("Save");
            if (menuItem == menuIdx + 4) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }
            yPosition += 9;
            // Load default settings
            display.setCursor(10, yPosition);
            display.print("Load Defaults");
            if (menuItem == menuIdx + 5) {
                display.drawTriangle(1, yPosition - 1, 1, yPosition + 7, 5, yPosition + 3, 1);
            }

            RedrawDisplay();
            return;
        }
    }
}

// Tap tempo function
static unsigned long lastTapTime = 0;
static unsigned long tapTimes[3] = {0, 0, 0};
static int tapIndex = 0;
void SetTapTempo() {
    if (usingExternalClock) {
        return;
    }
    unsigned long currentMillis = millis();
    if (currentMillis - lastTapTime > 2000) {
        tapIndex = 0;
    }
    if (tapIndex < 3) {
        tapTimes[tapIndex] = currentMillis;
        tapIndex++;
        lastTapTime = currentMillis;
    }
    if (tapIndex == 3) {
        unsigned long averageTime = (tapTimes[2] - tapTimes[0]) / 2;
        unsigned int newBPM = 60000 / averageTime;
        tapIndex++;
        UpdateBPM(newBPM);
        unsavedChanges = true;
    }
}

// Stop all outputs
void ToggleMasterStop() {
    masterStop = !masterStop;
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        outputs[i].ToggleMasterState();
    }
}

void HandleCVInputs() {
    oldChannelADC[0] = channelADC[0];
    oldChannelADC[1] = channelADC[1];
    channelADC[0] = analogRead(CV_1_IN_PIN) / ADCalibration[0];
    channelADC[1] = analogRead(CV_2_IN_PIN) / ADCalibration[1];

    // Use CV input 1 to control play/stop
}

// External clock interrupt service routine
void ClockReceived() {
    unsigned long currentTime = millis();
    unsigned long interval = currentTime - lastClockInterruptTime;
    lastClockInterruptTime = currentTime;

    static unsigned long intervals[3] = {0, 0, 0};
    static int intervalIndex = 0;

    intervals[intervalIndex] = interval;
    intervalIndex = (intervalIndex + 1) % 3;

    unsigned long averageInterval = (intervals[0] + intervals[1] + intervals[2]) / 3;

    // Divide the external clock signal by the selected divider
    if (externalTickCounter % externalClockDividers[externalDividerIndex] == 0) {
        if (averageInterval > 0) {
            clockInterval = averageInterval;
            unsigned int newBPM = 60000 / (averageInterval * externalClockDividers[externalDividerIndex]);
            if (abs(newBPM - BPM) > 3) { // Adjust BPM if the difference is significant
                UpdateBPM(newBPM);
                displayRefresh = 1;
                DEBUG_PRINT("External clock connected");
            }
        }
        for (int i = 0; i < NUM_OUTPUTS; i++) {
            outputs[i].SetExternalClock(true);
        }
        usingExternalClock = true;
        tickCounter = 0;
        ClockPulse(1);
    }
    externalTickCounter++;
}

void HandleExternalClock() {
    unsigned long currentTime = millis();
    if (usingExternalClock && (currentTime - lastClockInterruptTime) > 2000) {
        usingExternalClock = false;
        BPM = lastInternalBPM;
        UpdateBPM(BPM);
        for (int i = 0; i < NUM_OUTPUTS; i++) {
            outputs[i].SetExternalClock(false);
        }
        displayRefresh = 1;
        DEBUG_PRINT("External clock disconnected");
    }
}

void UpdateBPM(unsigned int newBPM) {
    BPM = constrain(newBPM, minBPM, maxBPM);
    TimerTcc0.setPeriod(60L * 1000 * 1000 / BPM / PPQN / 4);
}

void InitializeTimer() {
    // Set up the timer
    TimerTcc0.initialize();
    TimerTcc0.attachInterrupt(ClockPulseInternal);
}

void HandleOutputs() {
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        // Set the output level based on the pulse state
        if (outputs[i].GetPulseState()) {
            SetPin(i, outputs[i].GetOutputLevel());
        } else {
            SetPin(i, LOW);
        }
    }
}

void ClockPulse(int ppqn) { // Inside the interrupt
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        outputs[i].Pulse(ppqn, tickCounter);
    }
    tickCounter++;
}

void ClockPulseInternal() { // Inside the interrupt
    ClockPulse(PPQN);
}

void UpdateParameters(LoadSaveParams p) {
    BPM = p.BPM;
    externalDividerIndex = p.externalClockDivIdx;
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        outputs[i].SetDivider(p.divIdx[i]);
        outputs[i].SetDutyCycle(p.dutyCycle[i]);
        outputs[i].SetOutputState(p.outputState[i]);
        outputs[i].SetLevel(p.level[i]);
        outputs[i].SetSwingAmount(p.swingIdx[i]);
        outputs[i].SetSwingEvery(p.swingEvery[i]);
        outputs[i].SetPulseProbability(p.pulseProbability[i]);
        outputs[i].SetEuclidean(p.euclideanEnabled[i]);
        outputs[i].SetEuclideanSteps(p.euclideanSteps[i]);
        outputs[i].SetEuclideanTriggers(p.euclideanTriggers[i]);
        outputs[i].SetEuclideanRotation(p.euclideanRotations[i]);
        outputs[i].SetEuclideanPadding(p.euclideanPadding[i]);
        outputs[i].SetPhase(p.phaseShift[i]);
    }
}

void setup() {
    // Initialize serial port
    Serial.begin(115200);

    // Initialize I/O (DAC, pins, etc.)
    InitIO();

    // Initialize OLED display with address 0x3C for 128x64
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.drawBitmap(30, 0, VFM_Splash, 68, 64, 1);
    display.display();
    delay(2000);
    // Print module name in the middle of the screen
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(4, 20);
    display.print("ClockForge");
    display.setTextSize(1);
    display.setCursor(80, 54);
    display.print("V" VERSION);
    display.display();
    delay(1500);

    display.setTextColor(WHITE);

    // Attach interrupt for external clock
    attachInterrupt(digitalPinToInterrupt(CLK_IN_PIN), ClockReceived, RISING);

    // Load settings from flash memory or set defaults
    LoadSaveParams p = Load(0);
    UpdateParameters(p);

    // Initialize timer
    InitializeTimer();
    UpdateBPM(BPM);
}

void loop() {
    HandleEncoderClick();

    HandleEncoderPosition();

    HandleCVInputs();

    HandleExternalClock();

    HandleOutputs();

    HandleDisplay();
}
