//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                These functions are called by main.cpp                    //
//                           Make changes here                              //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#include "../include/robot.hpp"
#include "../include/defines.h"

RobotClass::RobotClass() :
    master(pros::E_CONTROLLER_MASTER),
    partner(pros::E_CONTROLLER_PARTNER)
{
    Drivetrain = new DrivetrainClass();
    Lift = new LiftClass();
}

//////////////////////////////////////////////////////////////////////////////
//                               Initialize                                 //
//                   Runs when the program first starts                     //
//////////////////////////////////////////////////////////////////////////////
void RobotClass::initialize() {
    Drivetrain->hardReset();
    master.rumble(". . .");
}

//////////////////////////////////////////////////////////////////////////////
//                                 Disabled                                 //
//               Only runs when the robot is in disabled mode               //
//////////////////////////////////////////////////////////////////////////////
void RobotClass::disabled() {}

//////////////////////////////////////////////////////////////////////////////
//                         Competition-Initialize                           //
//           Only runs when plugged into a comp/field controller            //
//////////////////////////////////////////////////////////////////////////////
void RobotClass::competition_initialize() {}

//////////////////////////////////////////////////////////////////////////////
//                                Autonomous                                //
//              Only runs when the robot is in autonomous mode              //
//////////////////////////////////////////////////////////////////////////////
void RobotClass::autonomous() {    
    Lift->moveUntimed(-127);
    Drivetrain->moveUntimed(100);
    pros::delay(900);
    Lift->moveUntimed(0);
    pros::delay(300);
    Drivetrain->moveUntimed(0);
    Drivetrain->reverse(true, false);
    Drivetrain->goToHeading(0, 250, 100);
    Drivetrain->reverse(false, true);
    Lift->move(127, 500);
    Drivetrain->move(-100, 750, 15);
    Drivetrain->reverse(true, false);
    Drivetrain->goToHeading(120, 500, 100);
    Drivetrain->reverse(false, true);
    Drivetrain->move(100, 600, 0);
    Lift->move(-127, 100);
    Drivetrain->move(-100, 600, 15);
    Drivetrain->reverse(true, false);
    Drivetrain->goToHeading(322, 500, 100);
    Drivetrain->reverse(false, true);
    Drivetrain->move(100, 1000, 0);
    Lift->move(127, 900);
    Drivetrain->move(-100, 1000, 15);
    Drivetrain->reverse(true, false);
    Drivetrain->goToHeading(120, 500, 100);
    Drivetrain->reverse(false, true);
}

//////////////////////////////////////////////////////////////////////////////
//                             Operator Control                             //
//               Only runs when the robot is in opcontrol mode              //
//////////////////////////////////////////////////////////////////////////////
void RobotClass::opcontrol() {
    // Send 'leftVal' and 'rightVal' to the drivetrain motors
    Drivetrain->update(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
}