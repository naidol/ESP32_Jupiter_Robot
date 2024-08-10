#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

#define MANUAL 0
#define AUTOMATIC 1

class PIDController {
public:
    PIDController(double* input, double* output, double* setpoint, double kp, double ki, double kd, int mode);

    void SetMode(int mode);
    void Compute();

private:
    double *input;
    double *output;
    double *setpoint;
    double kp, ki, kd;
    double integral, previous_error;
    bool inAuto;
};

#endif // PID_CONTROLLER_H
