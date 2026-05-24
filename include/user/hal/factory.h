#pragma once

#include <memory>
#include <string>

#include "user/hal/clock.h"
#include "user/hal/imu_backend.h"
#include "user/hal/joystick_backend.h"
#include "user/hal/motor_backend.h"

namespace qmini {
namespace hal {

// Backend selection is resolved at link time. Each backend's
// CMake target provides the concrete definitions of these factories.
//
// HardwareConfig is passed through opaquely: hardware backend uses it,
// sim/replay ignore it.

struct HardwareConfig {
    // network interface (e.g. "eth0", "wlP1p1s0") for Unitree DDS
    std::string network_interface = "eth0";
    // IMU serial port path
    std::string imu_serial_path =
        "/dev/serial/by-id/"
        "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
    int imu_baudrate = 921600;
    // joystick device
    std::string joystick_path = "/dev/input/js0";
    // motor startq from config.yaml
    float startq[10] = {};
};

std::unique_ptr<IMotorBackend>   make_motor_backend(const HardwareConfig& cfg);
std::unique_ptr<IImuBackend>     make_imu_backend(const HardwareConfig& cfg);
std::unique_ptr<IJoystickBackend> make_joystick_backend(const HardwareConfig& cfg);
std::unique_ptr<IClock>          make_clock();

}  // namespace hal
}  // namespace qmini
