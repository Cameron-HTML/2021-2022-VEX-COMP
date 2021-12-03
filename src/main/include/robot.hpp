#ifndef ROBOT_HPP
    #define ROBOT_HPP

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

// Include header files
#include "api.h"
#include "okapi/api.hpp"

#include "../include/drivetrain.hpp"
#include "../include/lift.hpp"

// Namespaces
using namespace std;
using namespace okapi;

class RobotClass {
     private:
        // Init controllers
        pros::Controller master;
        pros::Controller partner;

        // Init variables
        int threshold = 15;
        // -1 for mirror || 1 for normal
        int isMirrored = 1;
        int leftVal = 0;
        int rightVal = 0;
    public:
        // Init pointers
        DrivetrainClass* Drivetrain;
        LiftClass* Lift;

        RobotClass();
        static RobotClass* Get() {return TheRobot;}

        void autonomous();
  		void initialize();
  		void disabled();
  		void competition_initialize();
  		void opcontrol();
    protected:
        static RobotClass* TheRobot;
};

#endif // ROBOT_HPP
