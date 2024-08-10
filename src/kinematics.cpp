#include "kinematics.h"

// #define WHEEL_RADIUS 0.05   // Example wheel radius in meters
// #define WHEEL_BASE   0.3    // Example distance between left and right wheels in meters

WheelVelocities computeWheelVelocities(double linear_x, double angular_z) {
    WheelVelocities velocities;

    // Differential drive kinematics equations
    velocities.front_left  = linear_x - angular_z * (WHEEL_BASE / 2.0);
    velocities.front_right = linear_x + angular_z * (WHEEL_BASE / 2.0);
    velocities.back_left   = linear_x - angular_z * (WHEEL_BASE / 2.0);
    velocities.back_right  = linear_x + angular_z * (WHEEL_BASE / 2.0);

    return velocities;
}


// void calculateWheelVelocities(double linear_velocity, double angular_velocity,
//                               double wheel_base, double wheel_radius,
//                               double &setpoint_fl, double &setpoint_fr,
//                               double &setpoint_bl, double &setpoint_br)
// {
//     setpoint_fl = (linear_velocity - angular_velocity * wheel_base / 2.0) / wheel_radius;
//     setpoint_fr = (linear_velocity + angular_velocity * wheel_base / 2.0) / wheel_radius;
//     setpoint_bl = (linear_velocity - angular_velocity * wheel_base / 2.0) / wheel_radius;
//     setpoint_br = (linear_velocity + angular_velocity * wheel_base / 2.0) / wheel_radius;
// }

