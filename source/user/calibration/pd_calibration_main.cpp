// PD calibration tool entry point.
//
// Implementation of PD_CALIBRATION_SPEC.md (May 2026). Drives the HAL motor
// backend through the Test A/B/C protocol, captures per-tick traces to .npz
// under data/pd_calibration/<run_id>/, and emits manifest.json + run_meta.json.
//
// Refuses to start without --i-have-checked-the-harness. No ONNX, no policy,
// no walking control — pure low-level PD perturbation.

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <unistd.h>

#include "user/calibration/loop.h"
#include "user/calibration/trials.h"
#include "user/hal/factory.h"
#include "utils/config.h"

#ifdef QMINI_HAVE_VIEWER
#include "viewer.h"  // resolved via MUJOCO_PRIVATE_INC (mujoco backend only)
#endif

namespace fs = std::filesystem;
using qmini::calib::Trial;
using qmini::calib::TrialResult;

namespace {

qmini::calib::CalibrationLoop* g_loop = nullptr;
std::atomic<bool> g_abort{false};
void handle_sigint(int) { g_abort = true; if (g_loop) g_loop->stop(); }

void print_usage(const char* prog) {
    std::printf(
        "Usage: %s --i-have-checked-the-harness [options]\n"
        "\n"
        "PD calibration tool. Drives one joint at a time around the MGTO\n"
        "(stand pose, taken from config.yaml::ref_joint_act). All joints use\n"
        "their per-joint kp/kd from config.yaml (the deploy gains). No sweep,\n"
        "no policy loaded.\n"
        "\n"
        "Options:\n"
        "  -y, --yes               Skip the interactive 'Proceed?' confirmation\n"
        "                          (for sim / scripted runs). On hardware the\n"
        "                          default is to prompt before any motion.\n"
        "  --quick                 Single joint, single short step (~3 s).\n"
        "                          For sim sanity-checking the binary.\n"
        "  --tests A,B,C[,D,E]     Which tests to run (default: A,B,C). Opt-in:\n"
        "                          D = free-release passive dynamics (needs a\n"
        "                          gravity swing; FREE_RELEASE_CALIBRATION_SPEC.md);\n"
        "                          E = constant-velocity friction sweep (reads\n"
        "                          b,f from tau_est; works for any joint incl.\n"
        "                          hip_roll/hip_yaw/ankle). fit_friction.py.\n"
        "  --joints N,N,...        Which joints to run (default: all 10).\n"
        "  --sine-freqs <hz,...>   Test B frequencies to run, comma-separated\n"
        "                          (default 0.25,0.5,1,2,4,8). Use one value to\n"
        "                          do a single frequency per run.\n"
        "  --output-root <path>    Output root (default: data/pd_calibration).\n"
        "  --label <str>           Run label suffix (default: initial).\n"
        "  --tick-hz <hz>          Override control rate. Default is 1/control_dt\n"
        "                          from config.yaml (the deploy rate). Override\n"
        "                          prints a banner warning — the fitted PD model\n"
        "                          will be rate-specific.\n"
        "  --safe-dq-max <rad/s>   Velocity watchdog trip (default 4.0). Aborts\n"
        "                          the current trial if |dq| exceeds this for 2\n"
        "                          consecutive ticks. Lower = safer.\n"
        "  --ramp-in-s <s>         Smooth ramp from the measured pose to the\n"
        "                          stand pose (MGTO) before warm-up (default 3).\n"
        "                          Use 0 only if you are already holding the\n"
        "                          robot at the stand pose.\n"
        "  --no-fold               Don't fold (release gains to limp) when done.\n"
        "  --fold-secs <s>         Gain-release ramp at the end (default 2; 0 = instant).\n"
        "  --no-imu                Skip IMU backend (set imu_* fields to NaN).\n"
        "  --iface <name>          Network iface (hardware backend only).\n"
        "  --mjcf <path>           MuJoCo backend: which MJCF to load\n"
        "                          (default sim_assets/q1_sim.mjcf; pass\n"
        "                          sim_assets/q1_sim_hung.mjcf to dry-run\n"
        "                          the protocol with the robot pinned).\n"
        "  --viewer                Open the live GLFW viewer (mujoco only).\n"
        "                          Lets you watch the trial sequence before\n"
        "                          trusting it on a real robot.\n"
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
                    const std::vector<TrialResult>& results,
                    double tick_hz_requested,
                    double deploy_hz,
                    bool tick_hz_overridden) {
    std::ofstream f(run_dir / "manifest.json");
    f << "{\n";
    f << "  \"run_id\": \"" << json_escape(run_id) << "\",\n";
    f << "  \"robot\": \"Qmini Q1\",\n";
    f << "  \"sdk_commit\": \"" << json_escape(git_head_sha()) << "\",\n";
    f << "  \"tick_hz_requested\": " << tick_hz_requested << ",\n";
    f << "  \"deploy_hz\": " << deploy_hz << ",\n";
    f << "  \"tick_hz_overridden\": " << (tick_hz_overridden ? "true" : "false") << ",\n";
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
    bool assume_yes = false;
    bool quick = false;
    bool no_imu = false;
    std::string tests = "A,B,C";
    std::string joints_str;
    std::string sine_freqs_str;   // empty → default sweep {0.25..8}
    std::string output_root = "data/pd_calibration";
    std::string label = "initial";
    double tick_hz = -1.0;          // -1 sentinel → mirror 1/control_dt
    bool   tick_hz_overridden = false;
    double safe_dq_max = -1.0;      // -1 sentinel → use LoopOptions default (4.0)
    double ramp_in_s   = -1.0;      // -1 sentinel → use LoopOptions default (3.0)
    bool   no_fold = false;            // don't fold (release gains) at the end
    double fold_secs = -1.0;           // -1 sentinel → LoopOptions default (2.0)
    std::string iface = "eth0";
    std::string mjcf_path;
    bool viewer = false;
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
        else if (a == "--yes" || a == "-y") assume_yes = true;
        else if (a == "--quick") quick = true;
        else if (a == "--no-imu") no_imu = true;
        else if (a == "--tests")    tests = next("--tests");
        else if (a == "--joints")   joints_str = next("--joints");
        else if (a == "--sine-freqs") sine_freqs_str = next("--sine-freqs");
        else if (a == "--output-root") output_root = next("--output-root");
        else if (a == "--label")    label = next("--label");
        else if (a == "--tick-hz") {
            tick_hz = std::atof(next("--tick-hz"));
            tick_hz_overridden = true;
        }
        else if (a == "--safe-dq-max") {
            safe_dq_max = std::atof(next("--safe-dq-max"));
            if (safe_dq_max <= 0.0) {
                std::fprintf(stderr, "--safe-dq-max must be > 0 (was %g)\n", safe_dq_max);
                return 2;
            }
        }
        else if (a == "--ramp-in-s") {
            ramp_in_s = std::atof(next("--ramp-in-s"));
            if (ramp_in_s < 0.0) {
                std::fprintf(stderr, "--ramp-in-s must be >= 0 (was %g)\n", ramp_in_s);
                return 2;
            }
        }
        else if (a == "--no-fold") no_fold = true;
        else if (a == "--fold-secs") {
            fold_secs = std::atof(next("--fold-secs"));
            if (fold_secs < 0.0) {
                std::fprintf(stderr, "--fold-secs must be >= 0 (was %g)\n", fold_secs);
                return 2;
            }
        }
        else if (a == "--iface")    iface = next("--iface");
        else if (a == "--mjcf")     mjcf_path = next("--mjcf");
        else if (a == "--viewer")   viewer = true;
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

    // Resolve tick rate: default mirrors 1 / control_dt (the deploy rate);
    // explicit --tick-hz overrides but triggers a banner warning per spec §5.
    const double deploy_hz = (cfg.control_dt > 0.f) ? 1.0 / cfg.control_dt : 200.0;
    if (!tick_hz_overridden) {
        tick_hz = deploy_hz;
    } else {
        std::printf(
            "\n"
            "  *** WARNING: tick rate overridden to %.2f Hz. Deploy uses %.2f Hz from config.\n"
            "  *** The fitted PD model will be for %.2f Hz; applying it to a %.2f Hz sim/deploy\n"
            "  *** will introduce a discretization mismatch. Use the default rate unless you\n"
            "  *** are specifically characterizing rate-dependent behaviour.\n"
            "\n",
            tick_hz, deploy_hz, tick_hz, deploy_hz);
    }
    for (int i = 0; i < qmini::calib::kNumJoints; ++i) {
        mgto.q_target[i] = cfg.ref_joint_act[i];
    }
    std::array<float, qmini::calib::kNumJoints> lo{}, hi{};
    std::array<float, qmini::calib::kNumJoints> kp_arr{}, kd_arr{};
    for (int i = 0; i < qmini::calib::kNumJoints; ++i) {
        lo[i] = (i < static_cast<int>(cfg.act_pos_low.size())) ? cfg.act_pos_low[i] : -1.f;
        hi[i] = (i < static_cast<int>(cfg.act_pos_high.size())) ? cfg.act_pos_high[i] : +1.f;
        // Every test drives the joint at its deploy gain from config.yaml.
        kp_arr[i] = (i < static_cast<int>(cfg.kp.size())) ? cfg.kp[i] : 30.f;
        kd_arr[i] = (i < static_cast<int>(cfg.kd.size())) ? cfg.kd[i] : 1.f;
    }

    // Build plan.
    std::vector<Trial> plan;
    if (quick) {
        plan = qmini::calib::build_quick_plan(0);
    } else {
        // Test A is a single step trial; all tests use the per-joint deploy
        // gains (kp_arr/kd_arr from config.yaml) — no kp/kd sweep.
        // Test B frequencies: default sweep unless --sine-freqs given.
        std::vector<float> sine_freqs = {0.25f, 0.5f, 1.f, 2.f, 4.f, 8.f};
        if (!sine_freqs_str.empty()) {
            sine_freqs.clear();
            std::stringstream ss(sine_freqs_str);
            std::string tok;
            while (std::getline(ss, tok, ',')) {
                float f = std::atof(tok.c_str());
                if (f <= 0.f) {
                    std::fprintf(stderr, "--sine-freqs values must be > 0 (was '%s')\n",
                                 tok.c_str());
                    return 2;
                }
                sine_freqs.push_back(f);
            }
        }
        std::vector<Trial> all = qmini::calib::build_default_plan(
            lo, hi, mgto, kp_arr, kd_arr, sine_freqs);
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
             << " deploy_hz=" << deploy_hz
             << " tick_hz_overridden=" << (tick_hz_overridden ? "yes" : "no")
             << " quick=" << (quick ? "yes" : "no")
             << " plan_size=" << plan.size() << "\n";
    log_file.flush();

    // HAL backends.
    qmini::hal::HardwareConfig hw;
    hw.network_interface = iface;
    if (!mjcf_path.empty()) hw.mjcf_path = mjcf_path;
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
    if (safe_dq_max > 0.0) opts.safe_dq_max = static_cast<float>(safe_dq_max);
    if (quick) {
        opts.warm_up_s = 0.3;
        opts.rest_between_s = 0.2;
        opts.cooldown_s = 0.3;
        opts.ramp_in_s = 0.5;
        opts.fold_s = 0.5;
    }
    // Explicit --ramp-in-s overrides both the default and the --quick value.
    if (ramp_in_s >= 0.0) opts.ramp_in_s = ramp_in_s;
    if (no_fold) opts.fold_at_end = false;
    if (fold_secs >= 0.0) opts.fold_s = fold_secs;
    // Per-joint hold gains from config.yaml. With training-matched values
    // (QMINI_STIFFNESS / QMINI_PD_DAMPING), non-test joints are held with
    // realistic authority. The spec's blanket 80/2 over-stiffens small-
    // inertia joints (ankle, foot) and excites high-freq chain modes in
    // explicit-Euler MuJoCo, causing watchdog aborts during dry-runs.
    for (int i = 0; i < qmini::calib::kNumJoints; ++i) {
        opts.hold_kp[i] = (i < static_cast<int>(cfg.kp.size())) ? cfg.kp[i] : opts.kp_hold;
        opts.hold_kd[i] = (i < static_cast<int>(cfg.kd.size())) ? cfg.kd[i] : opts.kd_hold;
    }

    qmini::calib::CalibrationLoop loop(motor.get(), imu.get(), opts, mgto);
    g_loop = &loop;
    std::signal(SIGINT,  handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    // Live viewer (mujoco backend only). Lets the operator watch the trial
    // sequence in sim before trusting it on the real robot.
    if (viewer) {
#ifdef QMINI_HAVE_VIEWER
        if (qmini::hal::mj::Viewer::instance().start()) {
            std::printf("[calib] viewer window opened. Close it or hit Ctrl-C to abort.\n");
        }
#else
        std::fprintf(stderr,
            "[calib] --viewer requested but binary built without GLFW. "
            "Re-build with libglfw3-dev installed.\n");
#endif
    }

    std::printf("[calib] run_id=%s plan=%zu trials, tick=%.1f Hz, "
                "safe_dq_max=%.1f rad/s, ramp_in=%.1f s → %s\n",
                run_id.c_str(), plan.size(), tick_hz, opts.safe_dq_max,
                opts.ramp_in_s, run_dir.string().c_str());
    log_file << "[calib] safe_dq_max=" << opts.safe_dq_max << " rad/s"
             << " ramp_in_s=" << opts.ramp_in_s << "\n";
    log_file.flush();

    // Pre-run confirmation gate. The next step (loop.run) is the first thing
    // that drives the motors with gains, so require an explicit 'y' unless
    // --yes was passed. Until this point the motors are limp (kp=0).
    if (!assume_yes) {
        std::vector<int> distinct_joints;
        for (const auto& t : plan) {
            if (std::find(distinct_joints.begin(), distinct_joints.end(), t.joint)
                == distinct_joints.end()) {
                distinct_joints.push_back(t.joint);
            }
        }
        std::printf("\n  About to drive the robot:\n");
        std::printf("    joints :");
        for (int j : distinct_joints)
            std::printf(" %d(%s)", j, qmini::calib::kJointNames[j]);
        std::printf("\n");
        std::printf("    trials : %zu   tick: %.1f Hz   ramp-in: %.1f s   "
                    "safe_dq_max: %.1f rad/s\n",
                    plan.size(), tick_hz, opts.ramp_in_s, opts.safe_dq_max);
        std::printf("    Check : feet off ground, harness on torso, "
                    "e-stop in reach.\n");
        std::printf("\n  Proceed? type 'y' then Enter (anything else aborts): ");
        std::fflush(stdout);

        std::string line;
        if (!std::getline(std::cin, line) || line.empty() ||
            (line[0] != 'y' && line[0] != 'Y')) {
            std::printf("[calib] aborted at confirmation — no motion commanded.\n");
            log_file << "[calib] aborted at confirmation\n";
            motor->stop();
            if (imu) imu->stop();
            return 0;
        }
        std::printf("[calib] confirmed — ramping to stand.\n");
        std::fflush(stdout);
    }

    // Ramp to the stand pose (MGTO) now and hold it. Doing this explicitly
    // (rather than inside loop.run) lets us pause for a second confirmation
    // while the robot is physically at MGTO — so the operator can verify the
    // startq calibration produced the correct stand pose before perturbations.
    loop.ramp_to_mgto(opts.ramp_in_s);
    if (g_abort) {
        std::printf("[calib] aborted during ramp to stand.\n");
        loop.fold(opts.fold_s);   // ramp gains down to limp
        motor->stop();
        if (imu) imu->stop();
        return 0;
    }
    if (!assume_yes) {
        std::printf("\n  Robot is now holding the stand pose (MGTO).\n");
        std::printf("  Verify it looks correct — joints at the stand pose, "
                    "nothing off.\n");
        std::printf("\n  Start calibration trials? type 'y' then Enter "
                    "(anything else aborts): ");
        std::fflush(stdout);
        std::string mgto_ans;
        if (!std::getline(std::cin, mgto_ans) || mgto_ans.empty() ||
            (mgto_ans[0] != 'y' && mgto_ans[0] != 'Y')) {
            std::printf("[calib] aborted at MGTO confirmation — no trials run.\n");
            log_file << "[calib] aborted at MGTO confirmation\n";
            loop.fold(opts.fold_s);   // gentle release from the stand pose
            motor->stop();
            if (imu) imu->stop();
            return 0;
        }
        std::printf("[calib] MGTO confirmed — running trials.\n");
        std::fflush(stdout);
    }

    auto t_start = std::chrono::steady_clock::now();
    auto results = loop.run(plan, /*do_ramp=*/false);
    auto t_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();

#ifdef QMINI_HAVE_VIEWER
    if (viewer) qmini::hal::mj::Viewer::instance().stop();
#endif
    motor->stop();
    if (imu) imu->stop();

    write_manifest(run_dir, run_id, mgto.q_target, plan, results,
                   tick_hz, deploy_hz, tick_hz_overridden);
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
