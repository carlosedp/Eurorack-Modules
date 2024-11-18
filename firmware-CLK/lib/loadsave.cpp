#pragma once

#include <Arduino.h>
// Use flash memory as eeprom
#include <FlashAsEEPROM.h>

#define NUM_OUTPUTS 4
#define NUM_SLOTS 4
#define SLOT_SIZE (sizeof(LoadSaveParams))

// Struct to hold params that are saved/loaded to/from EEPROM
struct LoadSaveParams {
    unsigned int BPM;
    int divIdx[NUM_OUTPUTS];
    int dutyCycle[NUM_OUTPUTS];
    bool outputState[NUM_OUTPUTS];
    int level[NUM_OUTPUTS];
    int swingIdx[NUM_OUTPUTS];
    int swingEvery[NUM_OUTPUTS];
    int pulseProbability[NUM_OUTPUTS];
    bool euclideanEnabled[NUM_OUTPUTS];
    int euclideanSteps[NUM_OUTPUTS];
    int euclideanTriggers[NUM_OUTPUTS];
    int euclideanRotations[NUM_OUTPUTS];
};

// Save data to flash memory
void Save(const LoadSaveParams &p, int slot) { // save setting data to flash memory
    if (slot < 0 || slot >= NUM_SLOTS)
        return;
    delay(100);
    noInterrupts();
    int idx = slot * SLOT_SIZE;
    EEPROM.write(idx++, p.BPM);
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        EEPROM.write(idx++, p.divIdx[i]);
        EEPROM.write(idx++, p.dutyCycle[i]);
        EEPROM.write(idx++, p.outputState[i]);
        EEPROM.write(idx++, p.level[i]);
        EEPROM.write(idx++, p.swingIdx[i]);
        EEPROM.write(idx++, p.swingEvery[i]);
        EEPROM.write(idx++, p.pulseProbability[i]);
        EEPROM.write(idx++, p.euclideanEnabled[i]);
        EEPROM.write(idx++, p.euclideanSteps[i]);
        EEPROM.write(idx++, p.euclideanTriggers[i]);
        EEPROM.write(idx++, p.euclideanRotations[i]);
    }
    EEPROM.commit();
    interrupts();
}

// Load default setting data
LoadSaveParams LoadDefaultParams() {
    LoadSaveParams p;
    p.BPM = 120;
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        p.divIdx[i] = 5;
        p.dutyCycle[i] = 50;
        p.outputState[i] = true;
        p.level[i] = 100;
        p.swingIdx[i] = 0;
        p.swingEvery[i] = 2;
        p.pulseProbability[i] = 100;
        p.euclideanEnabled[i] = false;
        p.euclideanSteps[i] = 10;
        p.euclideanTriggers[i] = 6;
        p.euclideanRotations[i] = 1;
    }
    return p;
}

// Load setting data from flash memory
LoadSaveParams Load(int slot) {
    if (slot < 0 || slot >= NUM_SLOTS)
        return LoadDefaultParams();
    LoadSaveParams p;
    noInterrupts();
    if (EEPROM.isValid() == 1) {
        delay(100);
        int idx = slot * SLOT_SIZE;
        p.BPM = EEPROM.read(idx++);
        for (int i = 0; i < NUM_OUTPUTS; i++) {
            p.divIdx[i] = EEPROM.read(idx++);
            p.dutyCycle[i] = EEPROM.read(idx++);
            p.outputState[i] = EEPROM.read(idx++);
            p.level[i] = EEPROM.read(idx++);
            p.swingIdx[i] = EEPROM.read(idx++);
            p.swingEvery[i] = EEPROM.read(idx++);
            p.pulseProbability[i] = EEPROM.read(idx++);
            p.euclideanEnabled[i] = EEPROM.read(idx++);
            p.euclideanSteps[i] = EEPROM.read(idx++);
            p.euclideanTriggers[i] = EEPROM.read(idx++);
            p.euclideanRotations[i] = EEPROM.read(idx++);
        }
    } else {
        // If no eeprom data, set default values
        p = LoadDefaultParams();
    }
    interrupts();
    return p;
}
