#ifndef DIFF_DRIVE_LIB_HPP
#define DIFF_DRIVE_LIB_HPP

#include <string>
#include <phidget21.h>
#include <mutex>

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double setpoint, double measured, double dt);
private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double last_error_ = 0.0;
};

class SSC32Driver {
public:
    SSC32Driver(const std::string& port, int baudrate);
    ~SSC32Driver();
    void send_commands(int pwm_left, int pwm_right);
private:
    int fd_;
};

class PhidgetEncoderWrapper {
public:
    PhidgetEncoderWrapper(int serial_number);
    ~PhidgetEncoderWrapper();
    int get_position();
private:
    CPhidgetEncoderHandle handle_;
};

#endif
