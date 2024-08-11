#include "odometry.h"
#include <rcl/rcl.h>
#include <nav_msgs/msg/odometry.h>

// Global odometry variables
double x = 0.0;
double y = 0.0;
double theta = 0.0;

extern rcl_publisher_t odom_publisher;

void initOdometry() {
    // Initialize odometry variables
    x = 0.0;
    y = 0.0;
    theta = 0.0;
}

void updateOdometry(int32_t encoder_fl, int32_t encoder_fr, int32_t encoder_bl, int32_t encoder_br, double dt) {
    // Convert encoder counts to distance traveled by each wheel
    double distance_fl = encoder_fl * WHEEL_RADIUS * 2 * PI / ENCODER_TICKS_PER_REV;
    double distance_fr = encoder_fr * WHEEL_RADIUS * 2 * PI / ENCODER_TICKS_PER_REV;
    double distance_bl = encoder_bl * WHEEL_RADIUS * 2 * PI / ENCODER_TICKS_PER_REV;
    double distance_br = encoder_br * WHEEL_RADIUS * 2 * PI / ENCODER_TICKS_PER_REV;

    // Calculate average linear velocity and angular velocity
    double v_linear = (distance_fl + distance_fr + distance_bl + distance_br) / 4.0 / dt;
    double v_angular = (distance_fr + distance_br - distance_fl - distance_bl) / (4.0 * WHEEL_BASE) / dt;

    // Update odometry
    x += v_linear * cos(theta) * dt;
    y += v_linear * sin(theta) * dt;
    theta += v_angular * dt;
}

void publishOdometry() {
    nav_msgs__msg__Odometry odom_msg;

    // Fill in the odometry message
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

    // Publish the odometry message
    rcl_ret_t ret_odom_ok = rcl_publish(&odom_publisher, &odom_msg, NULL);
}
