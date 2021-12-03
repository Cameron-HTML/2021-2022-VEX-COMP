#ifndef LIFT_HPP
    #define LIFT_HPP

// Include header files
#include "api.h"
#include "okapi/api.hpp"

// Namespaces
using namespace std;
using namespace okapi;

class LiftClass {
    private:
        pros::Motor liftMotor;

        std::shared_ptr<AsyncPositionController<double, double>> liftControl;

        int height = 0;
        const int heights[3] = {0, 1100, 1000};

        bool upPressed = false;
        bool downPressed = false;

    public:
        LiftClass();

        void move(int speed, int time);
        void moveUntimed(int speed);
        void update(bool up, bool down);
        void hardReset();
};

#endif // LIFT_HPP
