#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "jupiter_robot_config.h"

struct WheelVelocities {
    double front_left;
    double front_right;
    double back_left;
    double back_right;
};

// Function to compute wheel velocities from robot linear and angular velocity
WheelVelocities computeWheelVelocities(double linear_x, double angular_z);

#endif // KINEMATICS_H
