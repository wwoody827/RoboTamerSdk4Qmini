#pragma once

#include <cstdio>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <vector>

namespace qmini {

class RLController;

class DataReporter {
public:
    DataReporter() = default;
    ~DataReporter();

    DataReporter(const DataReporter&) = delete;
    DataReporter& operator=(const DataReporter&) = delete;

    void init(bool general, bool rl,
              const char* udp_broadcast_ip = "255.255.255.255",
              int udp_port = 9870);
    void close();
    void report_data(const RLController& rl);

private:
    void report_data_title();
    void push_data2buffer(const RLController& rl);

    std::mutex mu_;
    const char* general_file_ = "general.txt";
    const char* rl_file_      = "rl.txt";
    FILE* general_fp_ = nullptr;
    FILE* rl_fp_      = nullptr;

    std::vector<float> general_buffer_;
    std::vector<float> rl_buffer_;

    int udp_sock_    = -1;
    int udp_counter_ = 0;
    struct sockaddr_in udp_addr_{};
};

}  // namespace qmini
