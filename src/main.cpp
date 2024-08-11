#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <esp32-hal-ledc.h>
#include <stdio.h>               
#include <Wire.h>

#include "encoder.h"
#include "odometry.h"
#include "pid_controller.h"
#include "kinematics.h"
#include "imu_bno.h"
#include "oled_display.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <utility/imumaths.h>
#include <std_msgs/msg/string.h>

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

// Setup PID controller objects for each motor wheel
PIDController pid_fl(&input_fl, &output_fl, &setpoint_fl, 1.0, 0.1, 0.01, AUTOMATIC);
PIDController pid_fr(&input_fr, &output_fr, &setpoint_fr, 1.0, 0.1, 0.01, AUTOMATIC);
PIDController pid_bl(&input_bl, &output_bl, &setpoint_bl, 1.0, 0.1, 0.01, AUTOMATIC);
PIDController pid_br(&input_br, &output_br, &setpoint_br, 1.0, 0.1, 0.01, AUTOMATIC);

// ROS2 and Micro-ROS - Declaration
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_publisher_t odom_publisher;             // odom     publisher
nav_msgs__msg__Odometry odom_msg;
rcl_subscription_t cmd_vel_subscriber;      // cmd_vel  subscriber
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_publisher_t imu_publisher;              // imu      publisher
sensor_msgs__msg__Imu imu_msg;
rcl_subscription_t led_subscriber;          // led      subscriber
std_msgs__msg__String led_msg;



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
} // end cmd_vel callback

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
} // end drive motors

// Callback to service the led_msg to drive LED indicator
void led_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (strcmp(msg->data.data, "listen") == 0) {   // Turn ON ESP32 LED when Jupiter is listening
        digitalWrite(LED_PIN, HIGH);
    } 
    else {
        digitalWrite(LED_PIN, LOW);
    }
} // end led_callback

// ******************************************* TIMER CALLBACK ***************************************************
// This timer callback executes every n-secs as defined in the Void Loop Ros Spin part of the code
// This callback is required to publish data to the ROS topics (imu/data and odometry/unfiltered)
// This data is required by the Nav2 system for robot pose and navigation through a map
// The timer callback will also serve to publish other data in the future that needs to be computed within
// the micro-controller and sent to the ROS-2 host PC (environment) for processing.

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    
    // ***************  Read the BNO055 sensor data  ************************************
    // Get the BNO055 IMU calibration status
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    // Read the 9DOF sensor data
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    /* Read the current temperature from BNO055 */
    int8_t temp = bno.getTemp();

    // Read the BNO055 Quaternion
    imu::Quaternion quat = bno.getQuat();

    // Fill the ROS2 imu_msg with IMU data just read from above
    imu_msg.header.stamp.nanosec = (uint32_t)(millis() * 1e6);
    imu_msg.header.stamp.sec = (uint32_t)(millis() / 1000);
    imu_msg.header.frame_id.data = (char*)("imu_link");

    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();
    imu_msg.orientation.w = quat.w(); 

    imu_msg.angular_velocity.x = angVelocityData.gyro.x;
    imu_msg.angular_velocity.y = angVelocityData.gyro.y;
    imu_msg.angular_velocity.z = angVelocityData.gyro.z;

    imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
    imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
    imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;

    // Publish the ROS-2 IMU message
    rcl_ret_t ret_imu_ok = rcl_publish(&imu_publisher, &imu_msg, NULL);

    // Clear & Display OLED data for Temp, IMU and Calibration 
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    // Display the temp
    display.print("Temperature C: "); display.println(temp);
    // Display the IMU 
    display.print("oX: "); display.print(orientationData.acceleration.heading, 1); display.print(" | ");
    display.print("oY: "); display.println(orientationData.acceleration.pitch, 1);
    display.print("oZ: "); display.println(orientationData.acceleration.roll, 1);
    display.print("gX: "); display.print(gravityData.gyro.x, 1); display.print(" | ");
    display.print("gY: "); display.println(gravityData.gyro.y, 1);
    display.print("gZ: "); display.println(gravityData.gyro.z, 1);
    // Display the calibration status (Sys | Gyro | Accel | Magno)
    display.println();
    display.print("C: S="); display.print(system); display.print("|"); 
    display.print("G="); display.print(gyro); display.print("|");
    display.print("A="); display.print(accel); display.print("|");
    display.print("M="); display.println(mag);
    // Populate the Oled Display
    display.display();

    // Update the Encoders, Compute PIDs and Drive the Motors, and Update Odometry and Publish Odom 
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

}  // end timer callback


// ********************************************** VOID SETUP ************************************************************
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

    // Setup the micro-ROS link
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
        "/odom/unfiltered");

    // Create cmd_vel subscriber
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    // Create esp_led subscriber
    rclc_subscription_init_default(
        &led_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "esp_led");
    
    // Create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100), // Publish Imu and other publishers within timer_callback  every 100 ms
        timer_callback);


    // Create executor to handle subscriber msgs (e.g. cmd_vel) and timer_callback 
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    setup_imu();
    setup_oled_display();

}
// ******************************************** VOID LOOP **********************************************************
void loop() {
    // ROS executor spin
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(50);

}
