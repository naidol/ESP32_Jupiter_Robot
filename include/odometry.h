#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "jupiter_robot_config.h"

// Initialize odometry
void initOdometry();

// Update odometry based on encoder counts
void updateOdometry(int32_t encoder_fl, int32_t encoder_fr, int32_t encoder_bl, int32_t encoder_br, double dt);

// Publish odometry data to ROS2
void publishOdometry();

#endif // ODOMETRY_H
