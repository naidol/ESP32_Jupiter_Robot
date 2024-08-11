#include "encoder.h"

volatile int32_t encoder_count_fl = 0;
volatile int32_t encoder_count_fr = 0;
volatile int32_t encoder_count_bl = 0;
volatile int32_t encoder_count_br = 0;

// Interrupt service routines for each encoder
void IRAM_ATTR handleEncoderFL() {
    encoder_count_fl++;
}

void IRAM_ATTR handleEncoderFR() {
    encoder_count_fr++;
}

void IRAM_ATTR handleEncoderBL() {
    encoder_count_bl++;
}

void IRAM_ATTR handleEncoderBR() {
    encoder_count_br++;
}

void initEncoders() {
    pinMode(ENCODER_FL_PIN_A, INPUT);
    pinMode(ENCODER_FL_PIN_B, INPUT);
    pinMode(ENCODER_FR_PIN_A, INPUT);
    pinMode(ENCODER_FR_PIN_B, INPUT);
    pinMode(ENCODER_BL_PIN_A, INPUT);
    pinMode(ENCODER_BL_PIN_B, INPUT);
    pinMode(ENCODER_BR_PIN_A, INPUT);
    pinMode(ENCODER_BR_PIN_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_FL_PIN_A), handleEncoderFL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_FR_PIN_A), handleEncoderFR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_BL_PIN_A), handleEncoderBL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_BR_PIN_A), handleEncoderBR, RISING);
}

int32_t readEncoderFL() {
    return encoder_count_fl;
}

int32_t readEncoderFR() {
    return encoder_count_fr;
}

int32_t readEncoderBL() {
    return encoder_count_bl;
}

int32_t readEncoderBR() {
    return encoder_count_br;
}
