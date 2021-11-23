#ifndef DRIVETRAIN_HPP
    #define DRIVETRAIN_HPP

// Include header files
#include "api.h"

// Namespaces
using namespace pros;
using namespace pros::literals;
using namespace std;

class DrivetrainClass {
    private:
        Motor frontLeftMotor;
        Motor backLeftMotor;
        Motor frontRightMotor;
        Motor backRightMotor;

        Imu inertialSensor;
    public:
        DrivetrainClass();

        void driveTrainLeft(int speed);
        void driveTrainRight(int speed);
        void driveTrainTurn(int speed);

        void update(int leftVal, int rightVal);
        void hardReset();
};

#endif // DRIVETRAIN_HPP