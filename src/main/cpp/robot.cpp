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
}

//////////////////////////////////////////////////////////////////////////////
//                               Initialize                                 //
//                   Runs when the program first starts                     //
//////////////////////////////////////////////////////////////////////////////
void RobotClass::initialize() {
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
    std::shared_ptr<OdomChassisController> chassis = 
        ChassisControllerBuilder()
            .withMotors({drivetrainFrontLeft, drivetrainBackLeft}, {-drivetrainFrontRight, -drivetrainBackRight})
            .withDimensions(AbstractMotor::gearset::blue, {{4_in, 11.5_in}, imev5BlueTPR})
            .withOdometry()
            .buildOdometry();

    chassis->setState({0_in, 0_in, 0_deg});
    chassis->driveToPoint({0_ft, 1_ft});
}

//////////////////////////////////////////////////////////////////////////////
//                             Operator Control                             //
//               Only runs when the robot is in opcontrol mode              //
//////////////////////////////////////////////////////////////////////////////
void RobotClass::opcontrol() {
    // Send 'leftVal' and 'rightVal' to the drivetrain motors
    //Drivetrain->update(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
}