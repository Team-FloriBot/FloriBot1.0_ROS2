#include "base/diff_drive_lib.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream> // Wichtig für std::cout

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
    // SSC-32 Tipp: Das Anhängen von 'T10' sagt dem Board, 
    // dass die Bewegung in 10ms abgeschlossen sein soll (passend zu deinem 100Hz Loop)
    std::string cmd = "#0P" + std::to_string(pwm_left) + 
                      "#1P" + std::to_string(pwm_right) + "T10\r";
    
    ssize_t bytes_written = write(fd_, cmd.c_str(), cmd.length());
    
    if (bytes_written < 0) {
        // Hier könnte man einen ROS_ERROR-Logger einbinden, 
        // aber wir bleiben in der Library ROS-unabhängig
        perror("SSC32 Serial Write Error");
    }
}

// --- Phidgets Implementation ---
PhidgetEncoderWrapper::PhidgetEncoderWrapper(int serial_number) {
    CPhidgetEncoder_create(&handle_);
    CPhidget_open((CPhidgetHandle)handle_, serial_number);
    if(CPhidget_waitForAttachment((CPhidgetHandle)handle_, 2000) != 0) {
        throw std::runtime_error("Phidget Encoder nicht gefunden: " + std::to_string(serial_number));
    }
    else {
        std::cout << "[PhidgetLib] Encoder erfolgreich verbunden. Seriennummer: " 
                  << serial_number << std::endl;
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
