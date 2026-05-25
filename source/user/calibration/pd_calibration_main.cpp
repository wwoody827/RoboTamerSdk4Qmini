// PD calibration tool entry point.
//
// Implementation of PD_CALIBRATION_SPEC.md (May 2026). Drives the HAL motor
// backend through the Test A/B/C protocol, captures per-tick traces to .npz
// under data/pd_calibration/<run_id>/, and emits manifest.json + run_meta.json.
//
// Refuses to start without --i-have-checked-the-harness. No ONNX, no policy,
// no walking control — pure low-level PD perturbation.

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "user/calibration/loop.h"
#include "user/calibration/trials.h"
#include "user/hal/factory.h"
#include "utils/config.h"

namespace fs = std::filesystem;
using qmini::calib::Trial;
using qmini::calib::TrialResult;

namespace {

qmini::calib::CalibrationLoop* g_loop = nullptr;
void handle_sigint(int) { if (g_loop) g_loop->stop(); }

void print_usage(const char* prog) {
    std::printf(
        "Usage: %s --i-have-checked-the-harness [options]\n"
        "\n"
        "PD calibration tool. Drives one joint at a time around the MGTO\n"
        "(stand pose, taken from config.yaml::ref_joint_act). Other joints\n"
        "hold MGTO at kp_hold/kd_hold (default 80/2). No policy is loaded.\n"
        "\n"
        "Options:\n"
        "  --quick                 Single joint, single short step (~3 s).\n"
        "                          For sim sanity-checking the binary.\n"
        "  --tests A,B,C           Which tests to run (default: A,B,C).\n"
        "  --joints N,N,...        Which joints to run (default: all 10).\n"
        "  --output-root <path>    Output root (default: data/pd_calibration).\n"
        "  --label <str>           Run label suffix (default: initial).\n"
        "  --tick-hz <hz>          Control rate (default: 200).\n"
        "  --no-imu                Skip IMU backend (set imu_* fields to NaN).\n"
        "  --iface <name>          Network iface (hardware backend only).\n"
        "  --operator <str>        Operator name for run_meta.json.\n"
        "  --notes <str>           Free-text notes for run_meta.json.\n"
        "  -h, --help              This help.\n",
        prog);
}

std::string now_run_id(const std::string& label) {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm);
    return std::string(buf) + "_" + label;
}

std::string git_head_sha() {
    FILE* p = popen("git rev-parse HEAD 2>/dev/null", "r");
    if (!p) return "unknown";
    char buf[64];
    std::string out = "unknown";
    if (std::fgets(buf, sizeof(buf), p)) {
        out.assign(buf);
        while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
            out.pop_back();
    }
    pclose(p);
    return out.empty() ? "unknown" : out;
}

std::string json_escape(const std::string& s) {
    std::string o;
    o.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
            case '"':  o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n";  break;
            case '\r': o += "\\r";  break;
            case '\t': o += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char esc[8];
                    std::snprintf(esc, sizeof(esc), "\\u%04x", c);
                    o += esc;
                } else o += c;
        }
    }
    return o;
}

void write_manifest(const fs::path& run_dir,
                    const std::string& run_id,
                    const std::array<float, qmini::calib::kNumJoints>& mgto,
                    const std::vector<Trial>& plan,
                    const std::vector<TrialResult>& results) {
    std::ofstream f(run_dir / "manifest.json");
    f << "{\n";
    f << "  \"run_id\": \"" << json_escape(run_id) << "\",\n";
    f << "  \"robot\": \"Qmini Q1\",\n";
    f << "  \"sdk_commit\": \"" << json_escape(git_head_sha()) << "\",\n";
    f << "  \"joint_names\": [";
    for (int i = 0; i < qmini::calib::kNumJoints; ++i) {
        f << "\"" << qmini::calib::kJointNames[i] << "\"";
        if (i + 1 < qmini::calib::kNumJoints) f << ", ";
    }
    f << "],\n";
    f << "  \"mgto_pose\": [";
    for (int i = 0; i < qmini::calib::kNumJoints; ++i) {
        f << mgto[i];
        if (i + 1 < qmini::calib::kNumJoints) f << ", ";
    }
    f << "],\n";
    f << "  \"trials\": [\n";
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const auto& t = plan[i];
        const auto& r = results[i];
        f << "    {";
        f << "\"joint\": " << t.joint;
        f << ", \"joint_name\": \"" << qmini::calib::kJointNames[t.joint] << "\"";
        f << ", \"test\": \"" << static_cast<char>(t.test) << "\"";
        f << ", \"pose\": \"MGTO\"";
        f << ", \"kp\": " << t.kp;
        f << ", \"kd\": " << t.kd;
        if (t.freq_hz >= 0.f) f << ", \"freq_hz\": " << t.freq_hz;
        else                  f << ", \"freq_hz\": null";
        f << ", \"amplitude_rad\": " << t.amp;
        f << ", \"duration_s\": " << t.duration_s;
        f << ", \"tick_rate_hz_actual\": " << r.tick_rate_hz_actual;
        f << ", \"n_samples\": " << r.n_samples;
        f << ", \"file\": \"" << json_escape(r.file) << "\"";
        f << ", \"result\": \"" << json_escape(r.result) << "\"";
        f << "}";
        if (i + 1 < plan.size()) f << ",";
        f << "\n";
    }
    f << "  ]\n";
    f << "}\n";
}

void write_run_meta(const fs::path& run_dir,
                    const std::string& run_id,
                    const std::string& operator_name,
                    const std::string& notes,
                    bool harness_checked) {
    std::ofstream f(run_dir / "run_meta.json");
    f << "{\n";
    f << "  \"run_id\": \"" << json_escape(run_id) << "\",\n";
    f << "  \"operator\": \"" << json_escape(operator_name) << "\",\n";
    f << "  \"harness_checked\": " << (harness_checked ? "true" : "false") << ",\n";
    f << "  \"ambient_temp_c\": null,\n";
    f << "  \"battery_voltage_v\": null,\n";
    f << "  \"notes\": \"" << json_escape(notes) << "\",\n";
    f << "  \"tool_version\": \"1.0.0\"\n";
    f << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
    bool harness_checked = false;
    bool quick = false;
    bool no_imu = false;
    std::string tests = "A,B,C";
    std::string joints_str;
    std::string output_root = "data/pd_calibration";
    std::string label = "initial";
    double tick_hz = 200.0;
    std::string iface = "eth0";
    std::string operator_name;
    std::string notes;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s requires a value\n", opt);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { print_usage(argv[0]); return 0; }
        else if (a == "--i-have-checked-the-harness") harness_checked = true;
        else if (a == "--quick") quick = true;
        else if (a == "--no-imu") no_imu = true;
        else if (a == "--tests")    tests = next("--tests");
        else if (a == "--joints")   joints_str = next("--joints");
        else if (a == "--output-root") output_root = next("--output-root");
        else if (a == "--label")    label = next("--label");
        else if (a == "--tick-hz")  tick_hz = std::atof(next("--tick-hz"));
        else if (a == "--iface")    iface = next("--iface");
        else if (a == "--operator") operator_name = next("--operator");
        else if (a == "--notes")    notes = next("--notes");
        else {
            std::fprintf(stderr, "Unknown option: %s\n", a.c_str());
            print_usage(argv[0]);
            return 2;
        }
    }

    if (!harness_checked) {
        std::fprintf(stderr,
            "\n"
            "  *** safety: pass --i-have-checked-the-harness to start.\n"
            "  ***\n"
            "  ***   feet must be 3-5 cm above ground\n"
            "  ***   harness grips the torso, not the legs\n"
            "  ***   e-stop / power-down switch within reach\n"
            "  ***   no obstacles in the leg swing envelope\n"
            "\n");
        return 2;
    }

    // Load config.yaml. The tool runs from a CWD that has it (bin/, tests/fixtures/, etc).
    ConfigParams cfg;
    qmini::calib::PoseRef mgto;
    mgto.label = "MGTO";
    if (cfg.ref_joint_act.size() < qmini::calib::kNumJoints) {
        std::fprintf(stderr, "config.yaml ref_joint_act has only %zu entries (need 10)\n",
                     cfg.ref_joint_act.size());
        return 1;
    }
    for (int i = 0; i < qmini::calib::kNumJoints; ++i) {
        mgto.q_target[i] = cfg.ref_joint_act[i];
    }
    std::array<float, qmini::calib::kNumJoints> lo{}, hi{};
    for (int i = 0; i < qmini::calib::kNumJoints; ++i) {
        lo[i] = (i < static_cast<int>(cfg.act_pos_low.size())) ? cfg.act_pos_low[i] : -1.f;
        hi[i] = (i < static_cast<int>(cfg.act_pos_high.size())) ? cfg.act_pos_high[i] : +1.f;
    }

    // Build plan.
    std::vector<Trial> plan;
    if (quick) {
        plan = qmini::calib::build_quick_plan(0);
    } else {
        std::vector<Trial> all = qmini::calib::build_default_plan(lo, hi, mgto);
        // Filter by --joints
        std::vector<int> joint_filter;
        if (!joints_str.empty()) {
            std::stringstream ss(joints_str);
            std::string tok;
            while (std::getline(ss, tok, ',')) {
                joint_filter.push_back(std::atoi(tok.c_str()));
            }
        }
        // Filter by --tests
        std::string sel = tests;
        for (auto& c : sel) c = std::toupper(c);
        for (const auto& t : all) {
            char tk = static_cast<char>(t.test);
            if (sel.find(tk) == std::string::npos) continue;
            if (!joint_filter.empty() &&
                std::find(joint_filter.begin(), joint_filter.end(), t.joint)
                == joint_filter.end()) continue;
            plan.push_back(t);
        }
    }

    if (plan.empty()) {
        std::fprintf(stderr, "[calib] empty plan after filtering — nothing to run\n");
        return 1;
    }

    // Output dir
    std::string run_id = now_run_id(label);
    fs::path run_dir = fs::path(output_root) / run_id;
    fs::create_directories(run_dir);

    // Tee stdout to log.txt: simplest is to just record key events; full stdout
    // redirection is up to the operator (`./pd_calibration_tool ... | tee`).
    std::ofstream log_file(run_dir / "log.txt");
    log_file << "[calib] run_id=" << run_id << "\n";
    log_file << "[calib] sdk_commit=" << git_head_sha() << "\n";
    log_file << "[calib] tests=" << tests
             << " tick_hz=" << tick_hz
             << " quick=" << (quick ? "yes" : "no")
             << " plan_size=" << plan.size() << "\n";
    log_file.flush();

    // HAL backends.
    qmini::hal::HardwareConfig hw;
    hw.network_interface = iface;
    for (int i = 0; i < qmini::calib::kNumJoints; ++i) {
        hw.startq[i] = (i < static_cast<int>(cfg.startq.size())) ? cfg.startq[i] : 0.f;
    }

    auto motor = qmini::hal::make_motor_backend(hw);
    if (!motor || !motor->start()) {
        std::fprintf(stderr, "[calib] motor backend failed to start\n");
        return 1;
    }
    std::unique_ptr<qmini::hal::IImuBackend> imu;
    if (!no_imu) {
        imu = qmini::hal::make_imu_backend(hw);
        if (imu) {
            if (!imu->start()) {
                std::fprintf(stderr, "[calib] IMU failed to start; continuing without it\n");
                imu.reset();
            }
        }
    }

    qmini::calib::LoopOptions opts;
    opts.tick_hz = tick_hz;
    opts.output_dir = run_dir.string();
    opts.verbose = true;
    if (quick) {
        opts.warm_up_s = 0.3;
        opts.rest_between_s = 0.2;
        opts.cooldown_s = 0.3;
    }

    qmini::calib::CalibrationLoop loop(motor.get(), imu.get(), opts, mgto);
    g_loop = &loop;
    std::signal(SIGINT,  handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    std::printf("[calib] run_id=%s plan=%zu trials, tick=%.1f Hz → %s\n",
                run_id.c_str(), plan.size(), tick_hz,
                run_dir.string().c_str());
    auto t_start = std::chrono::steady_clock::now();
    auto results = loop.run(plan);
    auto t_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();

    motor->stop();
    if (imu) imu->stop();

    write_manifest(run_dir, run_id, mgto.q_target, plan, results);
    write_run_meta(run_dir, run_id, operator_name, notes, harness_checked);

    int n_ok = 0, n_abort = 0;
    for (const auto& r : results) {
        if (r.result == "ok") ++n_ok;
        else                  ++n_abort;
    }
    std::printf("[calib] done in %.1f s — ok=%d abort=%d\n",
                elapsed, n_ok, n_abort);
    log_file << "[calib] elapsed_s=" << elapsed
             << " ok=" << n_ok << " abort=" << n_abort << "\n";

    return n_abort == 0 ? 0 : 1;
}
