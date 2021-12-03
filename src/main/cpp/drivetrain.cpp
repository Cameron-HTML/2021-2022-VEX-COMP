// Include drivetrain header file
#include "../include/drivetrain.hpp"
#include "../include/defines.h"

DrivetrainClass::DrivetrainClass() :
    frontLeftMotor(drivetrainFrontLeft, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS),
    backLeftMotor(drivetrainBackLeft, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS),
    frontRightMotor(drivetrainFrontRight, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS),
    backRightMotor(drivetrainBackRight, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS),
    IMU(IMUSensor)
{
    frontLeftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    backLeftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    frontRightMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    backRightMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
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

void DrivetrainClass::move(int speed, int time, int diff) {
    frontLeftMotor.move(speed);
    frontRightMotor.move(speed - diff);
    backLeftMotor.move(speed);
    backRightMotor.move(speed - diff);
    pros::delay(time);
    frontLeftMotor.move(0);
    frontRightMotor.move(0);
    backLeftMotor.move(0);
    backRightMotor.move(0);
}

void DrivetrainClass::moveUntimed(int speed) {
    frontLeftMotor.move(speed);
    frontRightMotor.move(speed);
    backLeftMotor.move(speed);
    backRightMotor.move(speed);
}

void DrivetrainClass::driveTrainTurn(int speed) {
    frontLeftMotor.move(speed);
    backLeftMotor.move(speed);
    frontRightMotor.move(-speed);
    backRightMotor.move(-speed);
}

void DrivetrainClass::reverse(bool leftRev, bool rightRev) {
    frontLeftMotor.tare_position();
    frontRightMotor.tare_position();

    frontLeftMotor.set_reversed(leftRev);
    backLeftMotor.set_reversed(leftRev);
    frontRightMotor.set_reversed(rightRev);
    backRightMotor.set_reversed(rightRev);
    pros::delay(500);
}

float DrivetrainClass::getHeading() {
    return IMU.get_heading();
}

float DrivetrainClass::inchToTicks(float inch) {
    // Convert the inches to ticks
    float ticks = inch * wheelCircum - 50;

    // Return ticks
    return ticks;
}

void DrivetrainClass::hardReset() {
    IMU.reset();
    frontLeftMotor.tare_position();
    frontRightMotor.tare_position();

    while(IMU.is_calibrating()) pros::delay(10);
}

int DrivetrainClass::timerValue(float seconds) {
    // Convert seconds to milliseconds
    int milliSeconds = seconds * 1000;

    if(milliSeconds < 250) {
        milliSeconds = 250;
    }

    return milliSeconds;
}

// Function for the Turn PID control
void DrivetrainClass::goToHeading(float target, float waitTime, int maxPower) {
    cout << "PID start" << endl;

    // PID variables decloration and initialization
    float kP = 13.5;
    float kI = 0.01;
    float kD = 38.0; // 14.0

    float offset = 0;

    // Variable for the raw intergral value
    float intergralRaw = 0;

    // Variable so the robot know when it should use intergral
    float intergralActiveZone = 10;

    // Variable to limit the value of intergral
    float intergralPowerLimit = 50 / kI;

    // Variable to track last error
    float lastError = 0;

    // Final output power/speed
    int finalPower = 0;

    // Variable to let the PID know when to start the timer
    bool startTime = false;

    // Variable to log the time when the timer starts
    int currentTime = 0;

    int scale = 1;

    // Convert our waitTimer to milliseconds
    timerValue(waitTime);

    while(currentTime < waitTime) {
        //cout << "Running" << endl;
        // Calculate how far the robot is from the target

        // Scale error so PID constants arent as large
        float error = target - getHeading();
        //cout << "error :" << getHeading() << endl;

        // Calculate the proportion
        float proportion = kP * error;

        // Check if intergral should be used
        if(abs(error) < intergralActiveZone && error != 0) {
            intergralRaw += error;
        } else {
            intergralRaw = 0;
        }

        // Limit the raw intergral
        if(intergralRaw > intergralPowerLimit) {
            intergralRaw = intergralPowerLimit;
        } else if(intergralRaw < -intergralPowerLimit) {
            intergralRaw = -intergralPowerLimit;
        }

        // Scale intergral
        float intergral = kI * intergralRaw;

        // Calculate derivative
        float derivative = kD * (error - lastError);
        
        // Update last error
        lastError = error;

        // Set derivative to 0 if at target
        if(error == 0) {
            derivative = 0;
        }

        // Calculate the final power to send to the motors
        finalPower = proportion + intergral + derivative;

        if(abs(error) > 180) {
            scale = -1;
        } else if(abs(error) < 180) {
            scale = 1;
        }

        // Limit final power
        if(finalPower > maxPower) {
            finalPower = maxPower * scale;
        } else if(finalPower < -maxPower) {
            finalPower = -maxPower * scale;
        }

        //cout << "Final power: " << finalPower << endl;

        // Set motors to final power
        driveTrainTurn(finalPower);

        // Start the timer
        if(abs(error) < 10) {
            startTime = true;
        }

        // Set timer values
        if(startTime) {
            if(currentTime == 0) {
                currentTime = pros::millis();
                waitTime += currentTime;
                cout << "Wait time: " << waitTime << endl;
            }
            currentTime = pros::millis();
        }

        cout << error << endl;
        cout << "Final power: " << finalPower << endl;

        // Delay code to stop CPU hoggin
        pros::delay(20);
    }
    driveTrainTurn(0);
}