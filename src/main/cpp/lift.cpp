// Include drivetrain header file
#include "../include/lift.hpp"
#include "../include/defines.h"

LiftClass::LiftClass() :
    liftMotor(liftMotorPort, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS)
{
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftControl = AsyncPosControllerBuilder().withMotor(-liftMotorPort).build();
    liftControl->flipDisable(true);
}

void LiftClass::move(int speed, int time) {
    liftMotor.move(speed);
    pros::delay(time);
    liftMotor.move(0);
}

void LiftClass::moveUntimed(int speed) {
    liftMotor.move(speed);
}

void LiftClass::update(bool up, bool down) {
    if(up && !upPressed) {
        liftControl->flipDisable(false);
        liftControl->setTarget(heights[1]);

        upPressed = up;
    } else if(down) {
        liftControl->flipDisable(true);

        liftMotor.move(-80);
    } else if(!up ){
        upPressed = false;
    } else if(liftControl->isDisabled()) {
        liftMotor.move(0);
    }
}
