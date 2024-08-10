#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <esp32-hal-ledc.h>
#include <stdio.h>               
#include <Wire.h>

#include "encoder.h"
#include "odometry.h"
#include "pid_controller.h"
#include "kinematics.h"
#include "bno_imu.h"
#include "oled_display.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// Motor pin definitions
#define MOTOR_FL_PWM  32  // Front-left motor PWM pin
#define MOTOR_FR_PWM  33  // Front-right motor PWM pin
#define MOTOR_BL_PWM  25  // Back-left motor PWM pin
#define MOTOR_BR_PWM  26  // Back-right motor PWM pin

// Motor direction pin definitions
#define MOTOR_FL_DIR  27
#define MOTOR_FR_DIR  14
#define MOTOR_BL_DIR  12
#define MOTOR_BR_DIR  13

// Motor control variables
double setpoint_fl = 0.0, input_fl = 0.0, output_fl = 0.0;
double setpoint_fr = 0.0, input_fr = 0.0, output_fr = 0.0;
double setpoint_bl = 0.0, input_bl = 0.0, output_bl = 0.0;
double setpoint_br = 0.0, input_br = 0.0, output_br = 0.0;

PIDController pid_fl(&input_fl, &output_fl, &setpoint_fl, 1.0, 0.1, 0.01, AUTOMATIC);
PIDController pid_fr(&input_fr, &output_fr, &setpoint_fr, 1.0, 0.1, 0.01, AUTOMATIC);
PIDController pid_bl(&input_bl, &output_bl, &setpoint_bl, 1.0, 0.1, 0.01, AUTOMATIC);
PIDController pid_br(&input_br, &output_br, &setpoint_br, 1.0, 0.1, 0.01, AUTOMATIC);

// ROS2 and micro-ROS related variables
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t odom_publisher;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// Function to handle incoming velocity commands
void cmdVelCallback(const void * msg_in) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msg_in;

    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // Compute wheel velocities using kinematics
    WheelVelocities wheel_vels = computeWheelVelocities(linear_x, angular_z);

    // Setpoints for PID controllers based on computed wheel velocities
    setpoint_fl = wheel_vels.front_left;
    setpoint_fr = wheel_vels.front_right;
    setpoint_bl = wheel_vels.back_left;
    setpoint_br = wheel_vels.back_right;
}

// Function to drive the motors based on PID output
void driveMotors(double output_fl, double output_fr, double output_bl, double output_br) {
    // Drive front-left motor
    digitalWrite(MOTOR_FL_DIR, output_fl >= 0 ? HIGH : LOW);
    ledcWrite(0, abs(output_fl));

    // Drive front-right motor
    digitalWrite(MOTOR_FR_DIR, output_fr >= 0 ? HIGH : LOW);
    ledcWrite(1, abs(output_fr));

    // Drive back-left motor
    digitalWrite(MOTOR_BL_DIR, output_bl >= 0 ? HIGH : LOW);
    ledcWrite(2, abs(output_bl));

    // Drive back-right motor
    digitalWrite(MOTOR_BR_DIR, output_br >= 0 ? HIGH : LOW);
    ledcWrite(3, abs(output_br));
}

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Initialize encoders
    initEncoders();

    // Initialize motor control pins
    pinMode(MOTOR_FL_DIR, OUTPUT);
    pinMode(MOTOR_FR_DIR, OUTPUT);
    pinMode(MOTOR_BL_DIR, OUTPUT);
    pinMode(MOTOR_BR_DIR, OUTPUT);

    // Setup PWM channels for motors
    ledcSetup(0, 5000, 8);  // Channel 0, 5kHz frequency, 8-bit resolution
    ledcAttachPin(MOTOR_FL_PWM, 0);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(MOTOR_FR_PWM, 1);
    ledcSetup(2, 5000, 8);
    ledcAttachPin(MOTOR_BL_PWM, 2);
    ledcSetup(3, 5000, 8);
    ledcAttachPin(MOTOR_BR_PWM, 3);

    // Initialize PID controllers
    pid_fl.SetMode(AUTOMATIC);
    pid_fr.SetMode(AUTOMATIC);
    pid_bl.SetMode(AUTOMATIC);
    pid_br.SetMode(AUTOMATIC);

    // Initialize odometry
    initOdometry();

    // // Initialize ROS node
    // rcl_allocator_t allocator = rcl_get_default_allocator();
    // rclc_support_t support;
    // rclc_support_init(&support, 0, NULL, &allocator);

    // // Create ROS node
    // rcl_node_t node;
    // rclc_node_init_default(&node, "esp32_robot_node", "", &support);

    // Initialize micro-ROS
    // set_microros_transports();
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();

    // Create init_options and support
    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node
    rclc_node_init_default(&node, "esp32_node", "", &support);

    // Create odometry publisher
    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom");

    // Create cmd_vel subscriber
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    // Create executor to handle cmd_vel messages
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);
}

void loop() {
    // ROS executor spin
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    // Time management for PID and odometry update
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    double dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Read encoder values
    int32_t encoder_fl = readEncoderFL();
    int32_t encoder_fr = readEncoderFR();
    int32_t encoder_bl = readEncoderBL();
    int32_t encoder_br = readEncoderBR();

    // Update inputs for PID controllers
    input_fl = encoder_fl / dt;
    input_fr = encoder_fr / dt;
    input_bl = encoder_bl / dt;
    input_br = encoder_br / dt;

    // Compute PID outputs
    pid_fl.Compute();
    pid_fr.Compute();
    pid_bl.Compute();
    pid_br.Compute();

    // Drive motors based on PID output
    driveMotors(output_fl, output_fr, output_bl, output_br);

    // Update odometry using encoder readings
    updateOdometry(encoder_fl, encoder_fr, encoder_bl, encoder_br, dt);

    // Publish odometry
    publishOdometry();
}
