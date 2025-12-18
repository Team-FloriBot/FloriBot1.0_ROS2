#include "base/diff_drive_lib.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>

// --- PID Implementation ---
PIDController::PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}

double PIDController::compute(double setpoint, double measured, double dt) {
    double error = setpoint - measured;
    integral_ += error * dt;
    double derivative = (error - last_error_) / dt;
    last_error_ = error;
    return (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
}

// --- SSC32 Implementation ---
SSC32Driver::SSC32Driver(const std::string& port, int baudrate) {
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) throw std::runtime_error("Fehler beim Öffnen des seriellen Ports");
    struct termios tty;
    tcgetattr(fd_, &tty);
    cfsetospeed(&tty, B115200); // Vereinfacht für dieses Beispiel
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tty);
}

SSC32Driver::~SSC32Driver() { if (fd_ >= 0) close(fd_); }

void SSC32Driver::send_commands(int pwm_left, int pwm_right) {
    std::string cmd = "#0P" + std::to_string(pwm_left) + "#1P" + std::to_string(pwm_right) + "\r";
    write(fd_, cmd.c_str(), cmd.length());
}

// --- Phidgets Implementation ---
PhidgetEncoderWrapper::PhidgetEncoderWrapper(int serial_number) {
    CPhidgetEncoder_create(&handle_);
    CPhidget_open((CPhidgetHandle)handle_, serial_number);
    if(CPhidget_waitForAttachment((CPhidgetHandle)handle_, 2000) != 0) {
        throw std::runtime_error("Phidget Encoder nicht gefunden: " + std::to_string(serial_number));
    }
}

PhidgetEncoderWrapper::~PhidgetEncoderWrapper() {
    CPhidget_close((CPhidgetHandle)handle_);
    CPhidget_delete((CPhidgetHandle)handle_);
}

int PhidgetEncoderWrapper::get_position() {
    int pos;
    CPhidgetEncoder_getPosition(handle_, 0, &pos);
    return pos;
}
