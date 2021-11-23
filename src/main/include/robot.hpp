#ifndef ROBOT_HPP
    #define ROBOT_HPP
    
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

// Include header files
#include "api.h"

#include "../include/drivetrain.hpp"

// Namespaces
using namespace pros;
using namespace pros::literals;

class RobotClass {
     private:
        // Init controllers
        Controller master;
        Controller partner;

        // Init variables
        int threshold = 15;
        // -1 for mirror || 1 for normal
        int isMirrored = 1;
        int leftVal = 0;
        int rightVal = 0;
    public:
        // Init pointers
        DrivetrainClass* Drivetrain;

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