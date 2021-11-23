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
    std::shared_ptr<AsyncPositionController<double, double>> fourBarControl =
        AsyncPosControllerBuilder().withMotor(-19).build();
    
    std::shared_ptr<AsyncPositionController<double, double>> fingerControl =
        AsyncPosControllerBuilder().withMotor(-12).build();

    std::shared_ptr<OdomChassisController> chassis = 
        ChassisControllerBuilder()
            .withMotors(-20, -11, -1, -10)
            .withDimensions(AbstractMotor::gearset::blue, {{4_in, 11.5_in}, imev5BlueTPR})
            .withOdometry()
            .buildOdometry();

    chassis->setMaxVelocity(600);
    chassis->setState({0_in, 0_in, 0_deg});
    fourBarControl->setTarget(150);
    chassis->moveDistanceAsync(8.7_ft);
    pros::delay(1000);
    fingerControl->setTarget(600);
    chassis->waitUntilSettled();
    chassis->moveDistance(-5.5_ft);
    chassis->setMaxVelocity(400);
    chassis->turnAngle(170_deg);
    fingerControl->setTarget(100);
    pros::delay(200);
    chassis->setMaxVelocity(600);
    chassis->moveDistance(1_ft);
    chassis->moveDistance(-1_ft);
    chassis->setMaxVelocity(400);
    chassis->turnAngle(-268_deg);
    chassis->setMaxVelocity(600);
    chassis->moveDistanceAsync(8.3_ft);
    pros::delay(1100);
    fingerControl->setTarget(600);
    chassis->waitUntilSettled();
    chassis->moveDistance(-8.5_ft);
}

//////////////////////////////////////////////////////////////////////////////
//                             Operator Control                             //
//               Only runs when the robot is in opcontrol mode              //
//////////////////////////////////////////////////////////////////////////////
void RobotClass::opcontrol() {
    // Send 'leftVal' and 'rightVal' to the drivetrain motors
    //Drivetrain->update(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
}