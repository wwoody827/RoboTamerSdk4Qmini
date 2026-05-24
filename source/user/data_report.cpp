#include "user/data_report.h"

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

#include "user/rl_controller.h"

namespace qmini {

namespace {
constexpr int kActJoints = 10;
}  // namespace

DataReporter::~DataReporter() { close(); }

void DataReporter::init(bool general, bool rl,
                        const char* udp_broadcast_ip, int udp_port) {
    if (general && general_fp_ == nullptr) {
        general_fp_ = std::fopen(general_file_, "w");
        if (!general_fp_) std::cerr << "general.txt: cannot open\n";
    }
    if (rl && rl_fp_ == nullptr) {
        rl_fp_ = std::fopen(rl_file_, "w");
        if (!rl_fp_) std::cerr << "rl.txt: cannot open\n";
    }
    udp_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock_ >= 0) {
        int broadcast = 1;
        setsockopt(udp_sock_, SOL_SOCKET, SO_BROADCAST,
                   &broadcast, sizeof(broadcast));
        std::memset(&udp_addr_, 0, sizeof(udp_addr_));
        udp_addr_.sin_family      = AF_INET;
        udp_addr_.sin_port        = htons(udp_port);
        udp_addr_.sin_addr.s_addr = inet_addr(udp_broadcast_ip);
        std::cout << "UDP broadcast → " << udp_broadcast_ip
                  << ":" << udp_port << "\n";
    }
    report_data_title();
}

void DataReporter::close() {
    std::lock_guard<std::mutex> g(mu_);
    if (general_fp_) { std::fclose(general_fp_); general_fp_ = nullptr; }
    if (rl_fp_)      { std::fclose(rl_fp_);      rl_fp_      = nullptr; }
    if (udp_sock_ >= 0) { ::close(udp_sock_); udp_sock_ = -1; }
}

void DataReporter::report_data_title() {
    if (general_fp_) {
        std::fprintf(general_fp_,
            "qC_l_hyaw\tqC_l_hrol\tqC_l_hpit\tqC_l_knee\tqC_l_apit\t"
            "qC_r_hyaw\tqC_r_hrol\tqC_r_hpit\tqC_r_knee\tqC_r_apit\t"
            "qP_l_hyaw\tqP_l_hrol\tqP_l_hpit\tqP_l_knee\tqP_l_apit\t"
            "qP_r_hyaw\tqP_r_hrol\tqP_r_hpit\tqP_r_knee\tqP_r_apit\t"
            "qV_l_hyaw\tqV_l_hrol\tqV_l_hpit\tqV_l_knee\tqV_l_apit\t"
            "qV_r_hyaw\tqV_r_hrol\tqV_r_hpit\tqV_r_knee\tqV_r_apit\t"
            "qT_l_hyaw\tqT_l_hrol\tqT_l_hpit\tqT_l_knee\tqT_l_apit\t"
            "qT_r_hyaw\tqT_r_hrol\tqT_r_hpit\tqT_r_knee\tqT_r_apit\t"
            "rol\tpit\tyaw\trol_rate\tpit_rate\tyaw_rate\t"
            "x_acc\ty_acc\tz_acc\t"
            "w_quat\tx_quat\ty_quat\tz_quat\n");
    }
}

void DataReporter::push_data2buffer(const RLController& rl) {
    std::vector<float> tmp;
    tmp.reserve(60);
    const auto& ja = rl.joint_act();
    const auto& jp = rl.joint_pos();
    const auto& jv = rl.joint_vel();
    const auto& jt = rl.joint_tau();
    for (int i = 0; i < kActJoints; ++i) tmp.push_back(ja(i));
    for (int i = 0; i < kActJoints; ++i) tmp.push_back(jp(i));
    for (int i = 0; i < kActJoints; ++i) tmp.push_back(jv(i));
    for (int i = 0; i < kActJoints; ++i) tmp.push_back(jt(i));
    for (int i = 0; i < 3; ++i) tmp.push_back(rl.base_rpy()(i));
    for (int i = 0; i < 3; ++i) tmp.push_back(rl.base_rpy_rate()(i));
    for (int i = 0; i < 3; ++i) tmp.push_back(rl.base_acc()(i));
    for (int i = 0; i < 4; ++i) tmp.push_back(rl.base_quat()(i));

    std::lock_guard<std::mutex> g(mu_);
    general_buffer_ = std::move(tmp);
    if (rl_fp_) {
        std::vector<float> rb;
        rb.reserve(rl.action_increment().size() + rl.observation().size());
        for (int i = 0; i < rl.action_increment().size(); ++i)
            rb.push_back(rl.action_increment()(i));
        for (int i = 0; i < rl.observation().size(); ++i)
            rb.push_back(rl.observation()(i));
        rl_buffer_ = std::move(rb);
    }
}

void DataReporter::report_data(const RLController& rl) {
    push_data2buffer(rl);
    std::lock_guard<std::mutex> g(mu_);
    if (general_fp_) {
        for (float v : general_buffer_) std::fprintf(general_fp_, "%lf\t", v);
        std::fprintf(general_fp_, "\n");
    }
    if (rl_fp_ && !rl_buffer_.empty()) {
        for (float v : rl_buffer_) std::fprintf(rl_fp_, "%lf\t", v);
        std::fprintf(rl_fp_, "\n");
    }
    if (udp_sock_ >= 0 && !general_buffer_.empty()) {
        if (++udp_counter_ >= 5) {
            udp_counter_ = 0;
            std::ostringstream oss;
            for (size_t i = 0; i < general_buffer_.size(); ++i) {
                if (i) oss << ',';
                oss << general_buffer_[i];
            }
            const std::string& msg = oss.str();
            sendto(udp_sock_, msg.c_str(), msg.size(), 0,
                   reinterpret_cast<sockaddr*>(&udp_addr_),
                   sizeof(udp_addr_));
        }
    }
}

}  // namespace qmini
