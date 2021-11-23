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

        shared_ptr<OdomChassisController> chassis;
    public:
        DrivetrainClass();

        void driveTrainLeft(int speed);
        void driveTrainRight(int speed);
        void driveTrainTurn(int speed);

        void update(int leftVal, int rightVal);
        void hardReset();
};

#endif // DRIVETRAIN_HPP
