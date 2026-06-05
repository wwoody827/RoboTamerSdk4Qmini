#include "user/flight_recorder.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace qmini {

namespace {

std::string utc_stamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm);
    return buf;
}

std::string vec_json(const std::vector<float>& v) {
    std::ostringstream o;
    o << '[';
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) o << ", ";
        o << v[i];
    }
    o << ']';
    return o.str();
}

}  // namespace

FlightRecorder::~FlightRecorder() { close(); }

bool FlightRecorder::open(const std::string& dir, const Meta& meta) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string stamp = utc_stamp();
    bin_path_  = dir + "/flight_" + stamp + ".bin";
    meta_path_ = dir + "/flight_" + stamp + ".meta.json";

    fp_ = std::fopen(bin_path_.c_str(), "wb");
    if (!fp_) {
        std::cerr << "[flight] cannot open " << bin_path_ << "\n";
        return false;
    }
    write_meta(meta);

    // ~90 s of slack at 66.7 Hz so disk hiccups never drop the fall window.
    cap_ = 6000;
    ring_.resize(cap_);
    head_ = tail_ = count_ = 0;
    running_ = true;
    open_ = true;
    writer_ = std::thread([this] { writer_loop(); });
    std::printf("[flight] recording -> %s\n", bin_path_.c_str());
    std::fflush(stdout);
    return true;
}

void FlightRecorder::log(const FlightRecord& rec) {
    if (!open_.load()) return;
    {
        std::lock_guard<std::mutex> g(mu_);
        if (count_ == cap_) {
            // Ring full: drop the OLDEST so the most recent (pre-crash) window
            // is always preserved.
            tail_ = (tail_ + 1) % cap_;
            --count_;
            dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        ring_[head_] = rec;
        head_ = (head_ + 1) % cap_;
        ++count_;
    }
    cv_.notify_one();
}

void FlightRecorder::writer_loop() {
    std::vector<FlightRecord> batch;
    batch.reserve(256);
    int since_flush = 0;
    while (true) {
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [this] { return count_ > 0 || !running_.load(); });
            if (count_ == 0 && !running_.load()) break;
            while (count_ > 0 && batch.size() < 256) {
                batch.push_back(ring_[tail_]);
                tail_ = (tail_ + 1) % cap_;
                --count_;
            }
        }
        if (!batch.empty() && fp_) {
            std::fwrite(batch.data(), sizeof(FlightRecord), batch.size(), fp_);
            since_flush += static_cast<int>(batch.size());
            batch.clear();
            // Flush ~10x/s so at most ~0.1 s is lost on a hard power cut.
            if (since_flush >= 7) { std::fflush(fp_); since_flush = 0; }
        }
    }
    if (fp_) std::fflush(fp_);
}

void FlightRecorder::close() {
    if (!open_.exchange(false)) {
        // Never opened, or already closed — still join any stray thread.
    }
    running_ = false;
    cv_.notify_all();
    if (writer_.joinable()) writer_.join();
    if (fp_) { std::fclose(fp_); fp_ = nullptr; }
    if (dropped_.load() > 0) {
        std::fprintf(stderr, "[flight] WARNING: dropped %llu records (disk too slow)\n",
                     static_cast<unsigned long long>(dropped_.load()));
    }
}

void FlightRecorder::write_meta(const Meta& meta) {
    std::FILE* mf = std::fopen(meta_path_.c_str(), "w");
    if (!mf) {
        std::cerr << "[flight] cannot open " << meta_path_ << "\n";
        return;
    }
    // numpy structured dtype, field order EXACTLY matching struct FlightRecord.
    // Parsed by tools/replay_flight.py: np.dtype(fields). itemsize must equal
    // sizeof(FlightRecord) (asserted on the Python side).
    std::ostringstream d;
    d << "[";
    d << "[\"t_mono\",\"<f8\"],";
    d << "[\"counter\",\"<i4\"],[\"mode\",\"<i4\"],[\"rl_counter\",\"<i4\"],[\"imu_valid\",\"<i4\"],";
    d << "[\"obs\",\"<f4\",[117]],[\"raw_action\",\"<f4\",[10]],";
    d << "[\"cmd_q\",\"<f4\",[10]],[\"cmd_kp\",\"<f4\",[10]],[\"cmd_kd\",\"<f4\",[10]],[\"cmd_tau_ff\",\"<f4\",[10]],";
    d << "[\"mot_q\",\"<f4\",[10]],[\"mot_dq\",\"<f4\",[10]],[\"mot_tau_est\",\"<f4\",[10]],";
    d << "[\"mot_tau_motor\",\"<f4\",[10]],[\"mot_temp\",\"<f4\",[10]],[\"mot_merror\",\"<i4\",[10]],";
    d << "[\"ctrl_q\",\"<f4\",[10]],[\"ctrl_dq\",\"<f4\",[10]],[\"joint_act\",\"<f4\",[10]],[\"ref_joint\",\"<f4\",[10]],";
    d << "[\"base_quat\",\"<f4\",[4]],[\"base_rpy\",\"<f4\",[3]],[\"base_omega\",\"<f4\",[3]],";
    d << "[\"base_acc\",\"<f4\",[3]],[\"command\",\"<f4\",[3]]";
    d << "]";

    std::ostringstream js;
    js << "{\n";
    js << "  \"schema\": \"qmini_flight/v1\",\n";
    js << "  \"record_size_bytes\": " << sizeof(FlightRecord) << ",\n";
    js << "  \"control_dt\": " << meta.control_dt << ",\n";
    js << "  \"policy_path\": \"" << meta.policy_path << "\",\n";
    js << "  \"sdk_commit\": \"" << meta.sdk_commit << "\",\n";
    js << "  \"joint_names\": [";
    for (size_t i = 0; i < meta.joint_names.size(); ++i) {
        if (i) js << ", ";
        js << "\"" << meta.joint_names[i] << "\"";
    }
    js << "],\n";
    js << "  \"dynamic_zero\": " << vec_json(meta.dynamic_zero) << ",\n";
    js << "  \"kp\": " << vec_json(meta.kp) << ",\n";
    js << "  \"kd\": " << vec_json(meta.kd) << ",\n";
    js << "  \"action_scale\": " << vec_json(meta.action_scale) << ",\n";
    js << "  \"ref_joint\": " << vec_json(meta.ref_joint) << ",\n";
    js << "  \"dtype\": " << d.str() << "\n";
    js << "}\n";

    std::fputs(js.str().c_str(), mf);
    std::fclose(mf);
}

}  // namespace qmini
