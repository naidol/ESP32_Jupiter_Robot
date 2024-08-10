#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "jupiter_robot_config.h"

// Initialize the encoders
void initEncoders();

// Read the encoder values for each wheel
int32_t readEncoderFL();
int32_t readEncoderFR();
int32_t readEncoderBL();
int32_t readEncoderBR();

#endif // ENCODER_H

