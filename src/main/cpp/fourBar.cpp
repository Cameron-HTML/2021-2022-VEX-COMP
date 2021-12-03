// Include drivetrain header file
#include "../include/fourBar.hpp"
#include "../include/defines.h"

FourBarClass::FourBarClass() :
    fourBarMotor(fourBarMotorPort, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS),
    finger(1, state)
{
}

void FourBarClass::update(bool up, bool down, bool open, bool close) {
    if(up) {
        fourBarMotor.move(127);
    } else if(down) {
        fourBarMotor.move(-127);
    } else {
        fourBarMotor.move(0);
    }

    if(open) {
        state = LOW;
    } else if(close) {
        state = HIGH;
    }

    if(state != lastState) {
        finger.set_value(state);
        
        lastState = state;
    }
}
