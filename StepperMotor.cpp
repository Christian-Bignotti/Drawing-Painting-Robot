#include "StepperMotor.h"

StepperMotor::StepperMotor(int step, int dir, int ena)
    : stepPin(step), dirPin(dir), enaPin(ena)
{
}

void StepperMotor::begin() {

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enaPin, OUTPUT);

    disable();
}

void StepperMotor::enable() {
    digitalWrite(enaPin, LOW);
}

void StepperMotor::disable() {
    digitalWrite(enaPin, HIGH);
}

void StepperMotor::step(bool direction, int delay_us) {

    digitalWrite(dirPin, direction);

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(stepPin, LOW);
}