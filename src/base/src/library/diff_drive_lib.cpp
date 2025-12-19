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
// Interner Callback für die Library
int CCONV AttachHandler(CPhidgetHandle phid, void *userptr) {
    int serial_number;
    CPhidget_getSerialNumber(phid, &serial_number);
    // Wir nutzen hier std::cout, da ROS_INFO in der Library nicht verfügbar ist
    std::cout << "[PhidgetLib] Hardware Attached: Serial " << serial_number <<  std::endl;
    return 0;
}

PhidgetEncoderWrapper::PhidgetEncoderWrapper(int expected_serial)
: handle_(nullptr)
{
    int result = 0;
    CPhidgetEncoderHandle h = nullptr;

    CPhidgetEncoder_create(&h);

    CPhidget_set_OnAttach_Handler(
        (CPhidgetHandle)h,
        AttachHandler,
        nullptr
    );

    // WICHTIG: wie im ROS1-Code
    CPhidget_open((CPhidgetHandle)h, -1);

    result = CPhidget_waitForAttachment((CPhidgetHandle)h, 10000);
    if (result != 0) {
        const char* err;
        CPhidget_getErrorDescription(result, &err);
        CPhidget_close((CPhidgetHandle)h);
        CPhidget_delete((CPhidgetHandle)h);
        throw std::runtime_error(
            "Phidget attach failed: " + std::string(err)
        );
    }

    int actual_serial = -1;
    CPhidget_getSerialNumber((CPhidgetHandle)h, &actual_serial);

    if (actual_serial != expected_serial) {
        CPhidget_close((CPhidgetHandle)h);
        CPhidget_delete((CPhidgetHandle)h);
        throw std::runtime_error(
            "Wrong Phidget attached. Expected "
            + std::to_string(expected_serial)
            + " got "
            + std::to_string(actual_serial)
        );
    }

    handle_ = h;
}


PhidgetEncoderWrapper::~PhidgetEncoderWrapper() {
    if (handle_) {
        CPhidget_close((CPhidgetHandle)handle_);
        CPhidget_delete((CPhidgetHandle)handle_);
        handle_ = nullptr;
    }
}

int PhidgetEncoderWrapper::get_position() {
    int pos = 0;
    // Index 0 ist der Standard-Encoder-Kanal bei PhidgetEncoder HighSpeed
    if (CPhidgetEncoder_getPosition(handle_, 0, &pos) != 0) {
        return last_known_pos_; 
    }
    last_known_pos_ = pos;
    return pos;
}

