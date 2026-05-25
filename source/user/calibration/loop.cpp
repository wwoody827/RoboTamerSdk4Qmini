#include "user/calibration/loop.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <thread>
#include <vector>

#include "user/calibration/npz_writer.h"

namespace fs = std::filesystem;

namespace qmini {
namespace calib {
namespace {

using clk = std::chrono::steady_clock;

// Sample buffer (in-memory, then dumped).
struct SampleBuf {
    std::vector<double> t_s;
    std::vector<float>  q_target;     // N*10 row-major
    std::vector<float>  dq_target;
    std::vector<float>  kp;
    std::vector<float>  kd;
    std::vector<float>  tau_ff;
    std::vector<float>  q;
    std::vector<float>  dq;
    std::vector<float>  tau_est;
    std::vector<float>  imu_rpy;      // N*3
    std::vector<float>  imu_omega;
    std::vector<float>  imu_acc;

    void reserve(std::size_t n) {
        t_s.reserve(n);
        q_target.reserve(n * kNumJoints);
        dq_target.reserve(n * kNumJoints);
        kp.reserve(n * kNumJoints);
        kd.reserve(n * kNumJoints);
        tau_ff.reserve(n * kNumJoints);
        q.reserve(n * kNumJoints);
        dq.reserve(n * kNumJoints);
        tau_est.reserve(n * kNumJoints);
        imu_rpy.reserve(n * 3);
        imu_omega.reserve(n * 3);
        imu_acc.reserve(n * 3);
    }
    void push_row(double t, const hal::MotorCmdFrame& cmd,
                  const hal::MotorStateFrame& st,
                  const hal::BaseStateFrame& base) {
        t_s.push_back(t);
        for (int i = 0; i < kNumJoints; ++i) {
            q_target.push_back(cmd.q_target[i]);
            dq_target.push_back(cmd.dq_target[i]);
            kp.push_back(cmd.kp[i]);
            kd.push_back(cmd.kd[i]);
            tau_ff.push_back(cmd.tau_ff[i]);
            q.push_back(st.q[i]);
            dq.push_back(st.dq[i]);
            tau_est.push_back(st.tau_est[i]);
        }
        if (base.valid) {
            for (int i = 0; i < 3; ++i) imu_rpy.push_back(base.rpy[i]);
            for (int i = 0; i < 3; ++i) imu_omega.push_back(base.omega[i]);
            for (int i = 0; i < 3; ++i) imu_acc.push_back(base.acc[i]);
        } else {
            for (int i = 0; i < 3; ++i) {
                imu_rpy.push_back(std::nanf(""));
                imu_omega.push_back(std::nanf(""));
                imu_acc.push_back(std::nanf(""));
            }
        }
    }
    std::size_t n() const { return t_s.size(); }
};

void write_npz(const std::string& path, const SampleBuf& buf,
               const Trial& trial,
               const PoseRef& mgto,
               double tick_rate_hz) {
    NpzWriter w(path);
    const std::size_t N = buf.n();
    w.add_f64_1d("t_s", buf.t_s.data(), N);
    w.add_f32_2d("q_target", buf.q_target.data(), N, kNumJoints);
    w.add_f32_2d("dq_target", buf.dq_target.data(), N, kNumJoints);
    w.add_f32_2d("kp", buf.kp.data(), N, kNumJoints);
    w.add_f32_2d("kd", buf.kd.data(), N, kNumJoints);
    w.add_f32_2d("tau_ff", buf.tau_ff.data(), N, kNumJoints);
    w.add_f32_2d("q", buf.q.data(), N, kNumJoints);
    w.add_f32_2d("dq", buf.dq.data(), N, kNumJoints);
    w.add_f32_2d("tau_est", buf.tau_est.data(), N, kNumJoints);
    w.add_f32_2d("imu_rpy", buf.imu_rpy.data(), N, 3);
    w.add_f32_2d("imu_omega", buf.imu_omega.data(), N, 3);
    w.add_f32_2d("imu_acc", buf.imu_acc.data(), N, 3);
    w.add_i64_scalar("joint_under_test", trial.joint);
    w.add_string_scalar("trial_label", trial.label);
    w.add_f32_1d("mgto_pose", mgto.q_target.data(), kNumJoints);
    w.add_f64_scalar("tick_rate_hz", tick_rate_hz);
    w.close();
}

}  // namespace

CalibrationLoop::CalibrationLoop(hal::IMotorBackend* motor,
                                 hal::IImuBackend*   imu,
                                 const LoopOptions&  opts,
                                 const PoseRef&      mgto_pose)
    : motor_(motor), imu_(imu), opts_(opts), mgto_(mgto_pose) {
    for (int i = 0; i < kNumJoints; ++i) {
        // Per-joint override if set, else scalar fallback.
        hold_kp_[i] = (opts_.hold_kp[i] > 0.f) ? opts_.hold_kp[i] : opts_.kp_hold;
        hold_kd_[i] = (opts_.hold_kd[i] > 0.f) ? opts_.hold_kd[i] : opts_.kd_hold;
    }
}

void CalibrationLoop::stop() { stop_requested_ = true; }

void CalibrationLoop::send_hold(double settle_s) {
    hal::MotorCmdFrame cmd{};
    for (int i = 0; i < kNumJoints; ++i) {
        cmd.q_target[i] = mgto_.q_target[i];
        cmd.kp[i] = hold_kp_[i];
        cmd.kd[i] = hold_kd_[i];
    }
    const long period_us = static_cast<long>(1e6 / opts_.tick_hz);
    auto next = clk::now();
    auto end  = clk::now() + std::chrono::duration_cast<clk::duration>(
                              std::chrono::duration<double>(settle_s));
    while (clk::now() < end && !stop_requested_) {
        motor_->send(cmd);
        motor_->read();  // keep the read side draining (mujoco needs this to step)
        next += std::chrono::microseconds(period_us);
        std::this_thread::sleep_until(next);
    }
}

void CalibrationLoop::cooldown() {
    if (opts_.verbose) std::printf("[calib] cooldown for %.1f s\n", opts_.cooldown_s);
    send_hold(opts_.cooldown_s);
}

std::vector<TrialResult> CalibrationLoop::run(const std::vector<Trial>& plan) {
    std::vector<TrialResult> results(plan.size());

    if (opts_.verbose) std::printf("[calib] warm-up %.1f s at MGTO\n", opts_.warm_up_s);
    send_hold(opts_.warm_up_s);

    const long period_us = static_cast<long>(1e6 / opts_.tick_hz);

    for (std::size_t idx = 0; idx < plan.size() && !stop_requested_; ++idx) {
        const Trial& tr = plan[idx];
        if (opts_.verbose) {
            std::printf("[calib] [%zu/%zu joint=%d (%s) test=%c kp=%.1f kd=%.2f"
                        " amp=%.3f dur=%.1fs] start\n",
                        idx + 1, plan.size(),
                        tr.joint, kJointNames[tr.joint],
                        static_cast<char>(tr.test),
                        tr.kp, tr.kd, tr.amp, tr.duration_s);
            std::fflush(stdout);
        }

        // Reserve capacity for the trial's samples.
        SampleBuf buf;
        std::size_t expected = static_cast<std::size_t>(tr.duration_s * opts_.tick_hz) + 8;
        buf.reserve(expected);

        hal::MotorCmdFrame cmd{};
        for (int i = 0; i < kNumJoints; ++i) {
            cmd.q_target[i] = mgto_.q_target[i];
            cmd.kp[i] = hold_kp_[i];
            cmd.kd[i] = hold_kd_[i];
        }
        cmd.kp[tr.joint] = tr.kp;
        cmd.kd[tr.joint] = tr.kd;

        auto trial_start = clk::now();
        auto trial_end   = trial_start + std::chrono::duration_cast<clk::duration>(
                                           std::chrono::duration<double>(tr.duration_s));
        auto next = trial_start;
        int  consecutive_overspeed = 0;
        std::string trial_result = "ok";

        // Rate watchdog: per spec §6 / §5, abort if achieved rate drops below
        // 90 % of the requested rate for more than 500 ms.
        std::deque<double> recent_ts;  // tick timestamps in the last 0.5 s
        double slow_since_s = -1.0;    // -1 = not currently slow
        const double rate_window_s = 0.5;
        const double rate_min_frac = 0.9;

        while (clk::now() < trial_end && !stop_requested_) {
            double t = std::chrono::duration<double>(clk::now() - trial_start).count();

            float off = trial_offset(tr, t);
            cmd.q_target[tr.joint] = mgto_.q_target[tr.joint] + off;

            motor_->send(cmd);
            hal::MotorStateFrame st = motor_->read();
            hal::BaseStateFrame  base = imu_ ? imu_->read() : hal::BaseStateFrame{};

            buf.push_row(t, cmd, st, base);

            // dq watchdog
            bool over = false;
            for (int i = 0; i < kNumJoints; ++i) {
                if (std::fabs(st.dq[i]) > opts_.safe_dq_max) { over = true; break; }
            }
            if (over) {
                ++consecutive_overspeed;
                if (consecutive_overspeed >= opts_.watchdog_n) {
                    trial_result = "aborted_watchdog";
                    if (opts_.verbose) {
                        std::printf("[calib]   watchdog: |dq|>%.1f rad/s for %d ticks\n",
                                    opts_.safe_dq_max, opts_.watchdog_n);
                    }
                    break;
                }
            } else {
                consecutive_overspeed = 0;
            }

            // Rate watchdog (after one full window is buffered).
            recent_ts.push_back(t);
            while (!recent_ts.empty() && recent_ts.front() < t - rate_window_s) {
                recent_ts.pop_front();
            }
            if (t >= rate_window_s) {
                const double expected = opts_.tick_hz * rate_window_s;
                const double actual   = static_cast<double>(recent_ts.size());
                const bool   is_slow  = actual < rate_min_frac * expected;
                if (is_slow) {
                    if (slow_since_s < 0) {
                        slow_since_s = t;
                    } else if (t - slow_since_s > rate_window_s) {
                        trial_result = "aborted_slow_rate";
                        if (opts_.verbose) {
                            double inst_hz = actual / rate_window_s;
                            std::printf("[calib]   rate watchdog: %.1f Hz < %.0f%% of %.1f Hz "
                                        "for >%.0f ms\n",
                                        inst_hz, rate_min_frac * 100, opts_.tick_hz,
                                        rate_window_s * 1000);
                        }
                        break;
                    }
                } else {
                    slow_since_s = -1.0;
                }
            }

            next += std::chrono::microseconds(period_us);
            std::this_thread::sleep_until(next);
        }

        if (stop_requested_ && trial_result == "ok") trial_result = "aborted_signal";

        // Measure actual rate.
        double trial_elapsed_s =
            std::chrono::duration<double>(clk::now() - trial_start).count();
        double rate_hz = (buf.n() > 1) ? (buf.n() - 1) / trial_elapsed_s : 0.0;

        // Write .npz under joint_NN_<name>/<label>.npz inside opts_.output_dir.
        char dir_name[64];
        std::snprintf(dir_name, sizeof(dir_name),
                      "joint_%02d_%s", tr.joint, kJointNames[tr.joint]);
        fs::path joint_dir = fs::path(opts_.output_dir) / dir_name;
        fs::create_directories(joint_dir);
        fs::path npz_path = joint_dir / (tr.label + ".npz");
        std::string rel = fs::path(dir_name) / (tr.label + ".npz");
        try {
            write_npz(npz_path.string(), buf, tr, mgto_, rate_hz);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[calib] failed to write %s: %s\n",
                         npz_path.string().c_str(), e.what());
            trial_result = std::string("write_error:") + e.what();
        }

        results[idx].file = rel;
        results[idx].result = trial_result;
        results[idx].tick_rate_hz_actual = rate_hz;
        results[idx].n_samples = static_cast<int>(buf.n());

        if (opts_.verbose) {
            std::printf("[calib]   stop %s (%zu samples, %.1f Hz actual)\n",
                        trial_result.c_str(), buf.n(), rate_hz);
        }

        // Return to MGTO + hold gains before next trial.
        send_hold(opts_.rest_between_s);
    }

    cooldown();
    return results;
}

}  // namespace calib
}  // namespace qmini
