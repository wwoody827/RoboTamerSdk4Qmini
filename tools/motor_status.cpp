// Motor status / ping tool — SAFE, zero-torque.
//
// Opens the same 4 FTDI serial ports as the hardware motor backend and sends
// each of the 10 GO-M8010-6 motors a command with kp=kd=tau=dq=0. With zero
// gains and zero feed-forward torque the motor produces NO torque, so this
// cannot move the robot even if the power bus is live. It only reads back each
// motor's reply and prints whether it answered, plus temp / error / position.
//
// Use it to distinguish "serial port / wiring problem" from "motor bus
// unpowered or e-stopped" when run_interface / pd_calibration_tool report
// "motor id=N does not reply".
//
// Build: cmake --build build/robot-release --target motor_status
// Run:   ./bin/motor_status [--rounds N]   (default 5 rounds, ~per motor)

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {

struct SerialGroup {
    const char* port;
    std::vector<int> motor_ids;
};

// Must stay in sync with motor_unitree.cpp::kSerialGroups.
const std::array<SerialGroup, 4> kSerialGroups = {{
    {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if03-port0", {0, 5}},
    {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if02-port0", {1, 6}},
    {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if00-port0", {2, 3, 4}},
    {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if01-port0", {7, 8, 9}},
}};

const std::array<const char*, 10> kJointNames = {
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_r",
};

constexpr float kSpeedRatio = 6.33f;
constexpr float kGearRatio  = 3.0f;

bool is_special(int motor_id) { return motor_id == 1 || motor_id == 6; }

int channel_id(int motor_id) {
    if (motor_id == 1) return 0;
    if (motor_id >= 2 && motor_id <= 4) return motor_id - 2;
    if (motor_id == 5) return 1;
    if (motor_id == 6) return 1;
    if (motor_id >= 7 && motor_id <= 9) return motor_id - 7;
    return motor_id;  // 0 -> 0
}

}  // namespace

int main(int argc, char** argv) {
    int rounds = 5;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--rounds" && i + 1 < argc) rounds = std::atoi(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::printf("Usage: %s [--rounds N]\n"
                        "Zero-torque ping of all 10 motors. Cannot move the robot.\n",
                        argv[0]);
            return 0;
        }
    }

    std::printf("motor_status: zero-torque ping (kp=kd=tau=0), %d rounds\n", rounds);
    std::printf("%-4s %-12s %-6s %-7s %-7s %-9s\n",
                "id", "joint", "reply", "temp", "err", "q_raw");
    std::printf("--------------------------------------------------------\n");

    int total = 0, replied = 0;

    for (const auto& g : kSerialGroups) {
        std::unique_ptr<SerialPort> serial;
        try {
            serial = std::make_unique<SerialPort>(g.port);
        } catch (const std::exception& e) {
            std::printf("[port %s] open failed: %s\n", g.port, e.what());
            for (int id : g.motor_ids) {
                ++total;
                std::printf("%-4d %-12s %-6s %-7s %-7s %s\n",
                            id, kJointNames[id], "----", "-", "-", "port error");
            }
            continue;
        }
        for (int id : g.motor_ids) {
            ++total;
            const float ratio = is_special(id) ? (kSpeedRatio * kGearRatio)
                                               : kSpeedRatio;
            bool any_ok = false;
            MotorData last{};
            for (int r = 0; r < rounds; ++r) {
                MotorCmd c{};
                MotorData d{};
                c.motorType = MotorType::GO_M8010_6;
                c.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                c.id  = channel_id(id);
                c.kp  = 0.f;   // zero gains -> zero torque, no motion
                c.kd  = 0.f;
                c.tau = 0.f;
                c.dq  = 0.f;
                c.q   = 0.f;
                d.motorType = MotorType::GO_M8010_6;
                try {
                    serial->sendRecv(&c, &d);
                } catch (...) {
                    // swallow: a throw here means no/garbled reply this round
                }
                if (d.correct) { any_ok = true; last = d; }
            }
            if (any_ok) {
                ++replied;
                std::printf("%-4d %-12s %-6s %-7d %-7d %+.4f (rad/ratio=%.2f)\n",
                            id, kJointNames[id], "OK", last.temp, last.merror,
                            last.q / ratio, ratio);
            } else {
                std::printf("%-4d %-12s %-6s %-7s %-7s %s\n",
                            id, kJointNames[id], "----", "-", "-", "no reply");
            }
        }
    }

    std::printf("--------------------------------------------------------\n");
    std::printf("motor_status: %d/%d motors replied\n", replied, total);
    if (replied == 0) {
        std::printf("\nAll motors silent. Serial ports opened OK, so this is "
                    "almost certainly:\n"
                    "  - motor power bus OFF, or\n"
                    "  - e-stop engaged, or\n"
                    "  - motor driver boards unpowered.\n"
                    "Check power / e-stop, then re-run.\n");
    } else if (replied < total) {
        std::printf("\nPartial: some motors replied, some didn't. Likely a "
                    "per-leg power/wiring/connector issue on the silent ones.\n");
    }
    return replied == total ? 0 : 1;
}
