#pragma once

// Real-robot black-box recorder. Captures one fixed-layout record per control
// tick AT THE HAL BOUNDARY, so a single log holds everything needed to replay
// a run and diagnose a fall offline:
//   - the EXACT 117-dim obs fed to the policy + the 10-dim raw action out
//   - the command actually sent to the motors (q/kp/kd/tau_ff)
//   - the RAW motor feedback (q, dq, tau_est, tau_motor, temp, merror)
//   - the controller-frame joint state + IMU (quat/rpy/omega/acc) + command
//
// Design:
//   - log() is called on the control thread; it only memcpy's into a ring
//     buffer (no disk I/O, no allocation) so it adds no jitter to the loop.
//   - a dedicated writer thread drains the ring to disk in batches.
//   - output is <dir>/flight_<UTC>.bin (raw FlightRecord structs back-to-back)
//     plus <dir>/flight_<UTC>.meta.json describing the numpy dtype + config,
//     so tools/replay_flight.py can `np.fromfile` it directly.
//
// The recorder lives above the HAL, so the SAME logs are produced by the sim,
// mujoco, and hardware backends — enabling sim↔real log diffing.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace qmini {

// Fixed on-disk layout. POD, tightly packed; field order MUST match the dtype
// emitted by FlightRecorder::write_meta(). Counts: 10 = per-joint, in the
// canonical order HYL HRL HPL KL AL HYR HRR HPR KR AR.
#pragma pack(push, 1)
struct FlightRecord {
    double  t_mono;            // seconds since recorder open (steady_clock)
    int32_t counter;          // control tick index
    int32_t mode;             // FSM mode (the char, as int)
    int32_t rl_counter;       // rl_control() invocation count
    int32_t imu_valid;        // BaseStateFrame.valid (0/1)

    float   obs[117];         // EXACT ONNX input this tick (valid when mode==3)
    float   raw_action[10];   // EXACT ONNX output (raw, pre-scale)

    // Command actually SENT to the motor backend (post dynamic-zero add-back).
    float   cmd_q[10];
    float   cmd_kp[10];
    float   cmd_kd[10];
    float   cmd_tau_ff[10];

    // RAW motor feedback straight from the HAL (q is pre dynamic-zero subtract).
    float   mot_q[10];
    float   mot_dq[10];
    float   mot_tau_est[10];
    float   mot_tau_motor[10];
    float   mot_temp[10];
    int32_t mot_merror[10];

    // Controller-frame joint state (post dynamic-zero, filtered) + targets.
    float   ctrl_q[10];       // what obs/joint_pos saw
    float   ctrl_dq[10];
    float   joint_act[10];    // controller internal target (pre dynamic-zero)
    float   ref_joint[10];

    // IMU / base, canonical body frame as the controller saw it.
    float   base_quat[4];     // w, x, y, z (raw from backend)
    float   base_rpy[3];
    float   base_omega[3];    // gyro / base_ang_vel
    float   base_acc[3];
    float   command[3];       // vx, vy, yaw_rate
};
#pragma pack(pop)

class FlightRecorder {
public:
    FlightRecorder() = default;
    ~FlightRecorder();

    FlightRecorder(const FlightRecorder&) = delete;
    FlightRecorder& operator=(const FlightRecorder&) = delete;

    struct Meta {
        std::vector<std::string> joint_names;
        std::vector<float> dynamic_zero;   // 10
        std::vector<float> kp;             // 10
        std::vector<float> kd;             // 10
        std::vector<float> action_scale;   // 10
        std::vector<float> ref_joint;      // 10
        float control_dt = 0.f;
        std::string policy_path;
        std::string sdk_commit;
    };

    // Creates <dir> if needed, opens flight_<UTC>.bin + .meta.json, starts the
    // writer thread. Returns false (and stays disabled) on failure.
    bool open(const std::string& dir, const Meta& meta);

    // Enqueue one record. Non-blocking; safe to call from the control thread.
    // No-op if not open. Drops (and counts) only if the ring overflows.
    void log(const FlightRecord& rec);

    void close();
    bool is_open() const { return open_.load(); }
    uint64_t dropped() const { return dropped_.load(); }
    const std::string& path() const { return bin_path_; }

private:
    void writer_loop();
    void write_meta(const Meta& meta);

    std::FILE* fp_ = nullptr;
    std::string bin_path_;
    std::string meta_path_;

    // Single-producer / single-consumer ring buffer.
    std::vector<FlightRecord> ring_;
    std::size_t cap_ = 0;
    std::size_t head_ = 0;     // next write slot (producer)
    std::size_t tail_ = 0;     // next read slot (consumer)
    std::size_t count_ = 0;
    std::mutex mu_;
    std::condition_variable cv_;
    std::thread writer_;
    std::atomic<bool> open_{false};
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> dropped_{0};
};

}  // namespace qmini
