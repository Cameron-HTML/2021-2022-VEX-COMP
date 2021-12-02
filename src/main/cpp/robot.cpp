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
    Drivetrain->hardReset();
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
    Drivetrain->hardReset();

    std::shared_ptr<AsyncPositionController<double, double>> fourBarControl =
        AsyncPosControllerBuilder().withMotor(-15).build();
    
    std::shared_ptr<AsyncPositionController<double, double>> fingerControl =
        AsyncPosControllerBuilder().withMotor(-7).build();

    std::shared_ptr<OdomChassisController> chassis = 
        ChassisControllerBuilder()
            .withMotors(1, 5, 20, 11)
            .withDimensions(AbstractMotor::gearset::blue, {{4_in, 11.5_in}, imev5BlueTPR})
            .withOdometry()
            .buildOdometry();

    chassis->setMaxVelocity(600);
    chassis->setState({0_in, 0_in, 0_deg});
    fourBarControl->setTarget(60);
    chassis->moveDistanceAsync(8.5_ft);
    pros::delay(1000);
    fingerControl->setTarget(700);
    chassis->waitUntilSettled();
    chassis->moveDistance(-5.7_ft);
    chassis->setMaxVelocity(400);
    Drivetrain->goToHeading(55, 1000, 127);
    fingerControl->setTarget(100);
    pros::delay(200);
    chassis->setMaxVelocity(600);
    chassis->moveDistance(1_ft);
    chassis->moveDistance(-.5_ft);
    chassis->setMaxVelocity(400);
    Drivetrain->goToHeading(325, 2000, 127);
    chassis->setMaxVelocity(550);
    chassis->moveDistanceAsync(8.9_ft);
    pros::delay(1100);
    fingerControl->setTarget(700);
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
    cout << Drivetrain->getHeading() << endl;
}