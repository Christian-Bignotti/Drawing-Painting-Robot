
#pragma once
#include <Arduino.h>

class StepperMotor {

private:
    int stepPin;
    int dirPin;
    int enaPin;

public:

    StepperMotor(int step, int dir, int ena);
    void begin();
    void enable();
    void disable();
    void step(bool direction, int delay_us = 80);
};

