#include "pid_controller.h"

// #define MANUAL 0
// #define AUTOMATIC 1

PIDController::PIDController(double* input, double* output, double* setpoint, double kp, double ki, double kd, int mode)
    : input(input), output(output), setpoint(setpoint), kp(kp), ki(ki), kd(kd), inAuto(false), integral(0.0), previous_error(0.0) {
    SetMode(mode);
}

void PIDController::SetMode(int mode) {
    inAuto = (mode == AUTOMATIC);
}

void PIDController::Compute() {
    if (!inAuto) return;

    double error = *setpoint - *input;
    integral += error;
    double derivative = error - previous_error;
    *output = kp * error + ki * integral + kd * derivative;
    previous_error = error;
}
