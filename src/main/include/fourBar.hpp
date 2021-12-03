#ifndef FOURBAR_HPP
    #define FOURBAR_HPP

// Include header files
#include "api.h"
#include "okapi/api.hpp"

// Namespaces
using namespace std;
using namespace okapi;

class FourBarClass {
    private:
        pros::Motor fourBarMotor;
        pros::ADIDigitalOut finger;

        bool state = LOW;
        bool lastState = LOW;

    public:
        FourBarClass();

        void update(bool up, bool down, bool open, bool close);
        void hardReset();
};

#endif // FOURBAR_HPP
