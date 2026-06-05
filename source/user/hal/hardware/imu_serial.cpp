// VSISLab IMU serial backend — pure C++, no embedded Python.
//
// Frame format: 0xFC | type | len | sn | crc8 | crc16_H | crc16_L | payload
//   type=0x40 (IMU)  → 56-byte payload, 12 floats + 2 ints; uses accel[3..5]
//   type=0x41 (AHRS) → 48-byte payload, 10 floats + 2 ints; uses rpy + quat
//
// The Python source (bin/imu_receiver.py) opened the port on EVERY call.
// We hold it open across reads, which saves ~hundreds of µs per tick and
// fixes a latent bug at 333 Hz tick rate. CRC bytes are read but not yet
// validated, preserving prior behavior; can be tightened later.

#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

constexpr uint8_t kFrameHead = 0xFC;
constexpr uint8_t kTypeImu   = 0x40;
constexpr uint8_t kTypeAhrs  = 0x41;
constexpr int     kImuLen    = 0x38;  // 56 bytes
constexpr int     kAhrsLen   = 0x30;  // 48 bytes

class SerialImuBackend : public IImuBackend {
public:
    SerialImuBackend(std::string path, int baud)
        : path_(std::move(path)), baud_(baud) {}
    ~SerialImuBackend() override { stop(); }

    bool start() override {
        if (!open_port()) {
            std::cerr << "imu_serial: open failed: " << std::strerror(errno)
                      << "\n";
            return false;
        }
        running_ = true;
        thread_ = std::thread([this] { run(); });
        return true;
    }

    void stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    BaseStateFrame read() override {
        std::lock_guard<std::mutex> g(mu_);
        return frame_;
    }

private:
    bool open_port() {
        fd_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) return false;
        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }
        cfmakeraw(&tty);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;  // 100ms timeout per read
        // 921600 baud requires either B921600 (glibc) or BOTHER with
        // c_ispeed/c_ospeed. We use B921600 — supported by every Linux
        // we ship to.
        speed_t s = B921600;
        switch (baud_) {
            case 115200: s = B115200; break;
            case 230400: s = B230400; break;
            case 460800: s = B460800; break;
            case 921600: s = B921600; break;
            default: break;
        }
        cfsetispeed(&tty, s);
        cfsetospeed(&tty, s);
        return tcsetattr(fd_, TCSANOW, &tty) == 0;
    }

    bool read_exact(uint8_t* buf, size_t n) {
        size_t got = 0;
        while (got < n) {
            ssize_t r = ::read(fd_, buf + got, n - got);
            if (r > 0) { got += r; continue; }
            if (r < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            return false;
        }
        return true;
    }

    void run() {
        // Sliding-window resync on FRAME_HEAD. Accumulate IMU + AHRS into
        // a single state frame; publish when both fresh.
        BaseStateFrame pending;
        bool have_imu = false, have_ahrs = false;
        uint8_t byte;
        while (running_) {
            if (!read_exact(&byte, 1)) continue;
            if (byte != kFrameHead) continue;
            uint8_t header[6];
            if (!read_exact(header, 6)) continue;
            const uint8_t type = header[0];
            const uint8_t len  = header[1];
            // header[2]=sn, header[3]=crc8, header[4..5]=crc16 (unchecked)
            if (type == kTypeImu) {
                if (len != kImuLen) continue;
                uint8_t payload[kImuLen];
                if (!read_exact(payload, kImuLen)) continue;
                float f[12];
                std::memcpy(f, payload, sizeof(f));
                // imu floats[3..5] = accel XYZ (m/s²). The mount transform
                // (see AHRS branch) negates the X and Z axes.
                pending.acc[0] = -f[3];
                pending.acc[1] =  f[4];
                pending.acc[2] = -f[5];
                have_imu = true;
            } else if (type == kTypeAhrs) {
                if (len != kAhrsLen) continue;
                uint8_t payload[kAhrsLen];
                if (!read_exact(payload, kAhrsLen)) continue;
                float f[10];
                std::memcpy(f, payload, sizeof(f));
                // Emit the canonical robot/training body frame directly from
                // the HAL so everything above the HAL line is backend-agnostic
                // (the sim/MuJoCo backend already emits this frame). Two
                // composed transforms:
                //   (1) IMU-payload axis remap (mirrors imu_receiver.py)
                //   (2) IMU->body mount transform: 180° about Y, i.e. negate
                //       the X and Z axes of every vector/RPY component. This
                //       was previously applied in RLController as
                //       trans_axis = {-1, +1, -1}; folding it here keeps the
                //       net transform on real data identical while letting the
                //       controller drop all sign flips.
                // Mount negates X (roll) and Z (yaw); Y (pitch) is unchanged.
                pending.omega[0] = -f[1];           // -(RollSpeed)
                pending.omega[1] = -f[0];           // PitchSpeed: remap(-f[0]), mount keeps Y
                pending.omega[2] = -f[2];           // -(HeadingSpeed)
                pending.rpy[0]   = -f[4];           // -(Roll = ahrs[4])
                pending.rpy[1]   = -f[3];           // Pitch = -ahrs[3] (mount keeps Y)
                pending.rpy[2]   = -f[5];           // -(Yaw = ahrs[5])
                pending.quat[0]  =  f[6];           // qw (telemetry-only; raw IMU frame)
                pending.quat[1]  =  f[7];           // qx
                pending.quat[2]  =  f[8];           // qy
                pending.quat[3]  =  f[9];           // qz
                have_ahrs = true;
            } else {
                // Unsupported type — drain its payload to keep sync.
                uint8_t skip[256];
                if (len <= sizeof(skip)) read_exact(skip, len);
            }
            if (have_imu && have_ahrs) {
                pending.valid = true;
                std::lock_guard<std::mutex> g(mu_);
                frame_ = pending;
                have_imu = false;
                have_ahrs = false;
            }
        }
    }

    std::string path_;
    int baud_;
    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex mu_;
    BaseStateFrame frame_{};
};

}  // namespace

std::unique_ptr<IImuBackend> make_imu_backend(const HardwareConfig& cfg) {
    return std::make_unique<SerialImuBackend>(cfg.imu_serial_path,
                                              cfg.imu_baudrate);
}

}  // namespace hal
}  // namespace qmini
