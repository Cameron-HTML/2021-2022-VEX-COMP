// Include drivetrain header file
#include "../include/drivetrain.hpp"
#include "../include/defines.h"

DrivetrainClass::DrivetrainClass() :
    frontLeftMotor(drivetrainFrontLeft, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_COUNTS),
    backLeftMotor(drivetrainBackLeft, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_COUNTS),
    frontRightMotor(drivetrainFrontRight, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_COUNTS),
    backRightMotor(drivetrainBackRight, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_COUNTS),

    inertialSensor(inertialSensorPort)
{

}

void DrivetrainClass::hardReset() {
    inertialSensor.reset();
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