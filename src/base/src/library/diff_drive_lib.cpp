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
    std::cout << "[PhidgetLib] Hardware Attached: Serial " << serial_number << std::endl;
    return 0;
}

PhidgetEncoderWrapper::PhidgetEncoderWrapper(int serial_number) : handle_(nullptr) {
    CPhidgetEncoder_create(&handle_);

    // Setze den Attach Handler (wie im Original)
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)handle_, AttachHandler, NULL);

    // Öffne das Gerät mit der spezifischen Seriennummer
    CPhidget_open((CPhidgetHandle)handle_, serial_number);

    // Warte auf Verbindung (10 Sekunden wie im Original)
    int result;
    if ((result = CPhidget_waitForAttachment((CPhidgetHandle)handle_, 10000)) != 0) {
        const char *err_ptr;
        CPhidget_getErrorDescription(result, &err_ptr);
        std::string err_msg(err_ptr);
        CPhidget_delete((CPhidgetHandle)handle_);
        throw std::runtime_error("Phidget (SN " + std::to_string(serial_number) + ") Fehler: " + err_msg);
    }
}

PhidgetEncoderWrapper::~PhidgetEncoderWrapper() {
    if (handle_) {
        CPhidget_close((CPhidgetHandle)handle_);
        CPhidget_delete((CPhidgetHandle)handle_);
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

