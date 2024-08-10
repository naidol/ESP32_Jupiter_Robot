#ifndef BNO_IMU_H
#define BNO_IMU_H

// BNO055 IMU
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BusIO_Register.h>
#include <Wire.h>
#include <sensor_msgs/msg/imu.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup_imu() {
    // Initialize I2C communication
    Wire.begin(21, 22);

    // Initialize the BNO055 sensor
    if (!bno.begin()) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR!");
        while (1);
    }

    // Set up the BNO055 sensor
    bno.setExtCrystalUse(true);

}

#endif // BNO_IMU_H
