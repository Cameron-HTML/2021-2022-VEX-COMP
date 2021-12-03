#ifndef FOURBAR_HPP
    #define FOURBAR_HPP

// Include header files
#include "api.h"
#include "okapi/api.hpp"

// Namespaces
using namespace std;
using namespace okapi;

class LiftClass {
    private:
        pros::Motor liftMotor;

    public:
        LiftClass();

        void move(int speed, int time);
        void moveUntimed(int speed);
        void update();
        void hardReset();
};

#endif // FOURBAR_HPP
