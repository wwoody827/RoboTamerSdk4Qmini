#pragma once

// POD frames exchanged across the HAL boundary.
//
// These types deliberately do NOT depend on:
//   - unitree_sdk2 / DDS IDL types (LowCmd_, LowState_, …)
//   - Eigen — kept here as plain arrays so this header is cheap to include
//   - pthread / python / serial / pygame
//
// Everything above the HAL line consumes these PODs and is bit-for-bit
// portable between hardware, sim, and replay backends.

#include <array>
#include <cstdint>

namespace qmini {
namespace hal {

constexpr int kNumJoints = 10;

struct MotorCmdFrame {
    std::array<float, kNumJoints> q_target{};
    std::array<float, kNumJoints> dq_target{};
    std::array<float, kNumJoints> kp{};
    std::array<float, kNumJoints> kd{};
    std::array<float, kNumJoints> tau_ff{};
};

struct MotorStateFrame {
    std::array<float, kNumJoints> q{};
    std::array<float, kNumJoints> dq{};
    std::array<float, kNumJoints> tau_est{};
    std::array<float, kNumJoints> ddq{};
};

struct BaseStateFrame {
    // rpy[0]=roll, rpy[1]=pitch, rpy[2]=yaw (rad)
    std::array<float, 3> rpy{};
    // angular velocity in base frame (rad/s)
    std::array<float, 3> omega{};
    // linear acceleration in base frame (m/s^2)
    std::array<float, 3> acc{};
    // quaternion (w, x, y, z)
    std::array<float, 4> quat{1.f, 0.f, 0.f, 0.f};
    bool valid = false;
};

struct JoystickFrame {
    // Axes in [-1, 1]. Index convention matches pygame / kernel jsX
    // for a PS4-style controller:
    //   axis[0] = left stick X    axis[1] = left stick Y
    //   axis[2] = right stick X   axis[3] = right stick Y
    std::array<float, 4> axis{};
    // Hat (D-pad): each in {-1, 0, 1}
    std::array<int, 2> hat{};
    // Buttons:
    //   0=A 1=B 2=X 3=Y 4=L1 5=R1 6=L2 7=R2 8=SELECT 9=START
    std::array<int, 10> button{};
    bool valid = false;
};

}  // namespace hal
}  // namespace qmini
