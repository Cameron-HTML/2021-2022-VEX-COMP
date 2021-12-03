#ifndef DRIVETRAIN_HPP
    #define DRIVETRAIN_HPP

// Include header files
#include "api.h"
#include "okapi/api.hpp"

// Namespaces
using namespace std;
using namespace okapi;

class DrivetrainClass {
    private:
        pros::Motor frontLeftMotor;
        pros::Motor backLeftMotor;
        pros::Motor frontRightMotor;
        pros::Motor backRightMotor;

        pros::Imu IMU;
    public:
        DrivetrainClass();

        void driveTrainLeft(int speed);
        void driveTrainRight(int speed);
        void driveTrainTurn(int speed);

        void move(int speed, int time, int diff);
        void moveUntimed(int speed);

        int timerValue(float seconds);
        float inchToTicks(float inch);
        void goToHeading(float target, float waitTime, int maxPower);
        void driveDistance(float target, float waitTime, int maxPower, int rampTimeGiven);

        void reverse(bool leftRev, bool rightRev);
        
        float getHeading();

        void update(int leftVal, int rightVal);
        void hardReset();
};

#endif // DRIVETRAIN_HPP
