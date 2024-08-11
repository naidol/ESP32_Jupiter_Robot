// ****************************************************************************************
// FILE:    jupiter_robot_config.h
// PURPOSE: configuration file for ESP32 pin assignment, PID settings and global variables
// AUTHOR:  Logan Naidoo, South Africa, 2024
// ****************************************************************************************

// PID Controller Settings
#define K_P 0.6                             // P constant
#define K_I 0.8                             // I constant
#define K_D 0.5                             // D constant

// Define Onboard ESP32 LED
#define LED_PIN 2

// Define Jupiter Robot Motor Specs & Wheel Specs
#define MOTOR_MAX_RPM 333                   // motor's max RPM          
#define MAX_RPM_RATIO 0.85                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 1477                // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 1487                // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 1454                // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 1492                // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.090                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.400            // distance between left and right wheels
#define PWM_BITS 8                          // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 5000                  // PWM Frequency
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

// Jupiter Constants from GPT ------------------------------------------------------------------------------------

// Constants for LEDC channels
#define LEDC_CHANNEL_FL_FORWARD 0
#define LEDC_CHANNEL_FL_BACKWARD 1
#define LEDC_CHANNEL_FR_FORWARD 2
#define LEDC_CHANNEL_FR_BACKWARD 3
#define LEDC_CHANNEL_BL_FORWARD 4
#define LEDC_CHANNEL_BL_BACKWARD 5
#define LEDC_CHANNEL_BR_FORWARD 6
#define LEDC_CHANNEL_BR_BACKWARD 7

// Constants for LEDC PWM setup
#define LEDC_TIMER_13_BIT  8 //13
#define LEDC_BASE_FREQ     5000

// Define MOTOR & ENCODER pins for all four wheels
// Jupiter Robot ESP32 Pins Configuration
//                Front
// Motor #1 FL  <-| |->  FR  Motor #2
// Motor #3 FB  <-| |->  BR  Motor #4
//                BACK

// M1
#define MOTOR_FL_FORWARD 32      
#define MOTOR_FL_BACKWARD 33 
#define ENCODER_FL_PIN_A 25
#define ENCODER_FL_PIN_B 26   
// M2
#define MOTOR_FR_FORWARD 23         
#define MOTOR_FR_BACKWARD 19  
#define ENCODER_FR_PIN_A 18
#define ENCODER_FR_PIN_B 5
//M3
#define MOTOR_BL_FORWARD 27
#define MOTOR_BL_BACKWARD 14
#define ENCODER_BL_PIN_A 12
#define ENCODER_BL_PIN_B 13
//M4
#define MOTOR_BR_FORWARD 17
#define MOTOR_BR_BACKWARD 16
#define ENCODER_BR_PIN_A 4
#define ENCODER_BR_PIN_B 15

// Define encoder ticks per revolution
#define ENCODER_TICKS_PER_REV 360

// Wheel parameters
#define WHEEL_RADIUS 0.045      // in meters
#define WHEEL_BASE 0.400        // distance between right and left in meters
