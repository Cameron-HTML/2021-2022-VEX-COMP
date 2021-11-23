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

    public:
        FourBarClass();

        void update();
        void hardReset();
};

#endif // FOURBAR_HPP
