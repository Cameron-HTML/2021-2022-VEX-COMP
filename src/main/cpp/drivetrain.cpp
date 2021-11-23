// Include drivetrain header file
#include "../include/drivetrain.hpp"
#include "../include/defines.h"

DrivetrainClass::DrivetrainClass() :
    frontLeftMotor(drivetrainFrontLeft, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS),
    backLeftMotor(drivetrainBackLeft, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS),
    frontRightMotor(drivetrainFrontRight, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS),
    backRightMotor(drivetrainBackRight, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS)
{
}

void DrivetrainClass::update(int leftVal, int rightVal) {
    frontLeftMotor.move(leftVal);
    backLeftMotor.move(leftVal);
    frontRightMotor.move(rightVal);
    backRightMotor.move(rightVal);
}

void DrivetrainClass::driveTrainLeft(int speed) {
    frontLeftMotor.move(speed);
    backLeftMotor.move(speed);
}

void DrivetrainClass::driveTrainRight(int speed) {
    frontRightMotor.move(speed);
    backRightMotor.move(speed);
}

void DrivetrainClass::driveTrainTurn(int speed) {
    frontLeftMotor.move(speed);
    backLeftMotor.move(speed);
    frontRightMotor.move(-speed);
    backRightMotor.move(-speed);
}