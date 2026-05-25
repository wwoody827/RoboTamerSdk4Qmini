#pragma once

// Minimal in-process .npz writer. Output is exactly what
// numpy.load(file)[key] reads back.
//
// Scope: just enough for the PD calibration trial logs.
//   - 1-D / 2-D contiguous arrays in float32, float64, int32, int64
//   - scalars (shape ()) for int64 and float64
//   - scalar strings (stored as numpy Unicode `<UN`)
// No compression — entries are ZIP-stored. Trial logs are well under 1 MB
// each, so the overhead is irrelevant.

#include <cstdint>
#include <string>
#include <vector>

namespace qmini {
namespace calib {

class NpzWriter {
public:
    explicit NpzWriter(const std::string& path);
    ~NpzWriter();

    // Add a 2-D float32 array: shape (rows, cols), row-major.
    void add_f32_2d(const std::string& name,
                    const float* data,
                    std::size_t rows,
                    std::size_t cols);

    // Add a 1-D float64 array.
    void add_f64_1d(const std::string& name,
                    const double* data,
                    std::size_t n);

    // Add a 0-D scalar of int64 / float64.
    void add_i64_scalar(const std::string& name, std::int64_t v);
    void add_f64_scalar(const std::string& name, double v);

    // Add a 1-D float32 array (e.g. mgto_pose).
    void add_f32_1d(const std::string& name,
                    const float* data,
                    std::size_t n);

    // Add a scalar string (numpy treats as `<UN` where N = len).
    void add_string_scalar(const std::string& name,
                           const std::string& value);

    // Flush ZIP central directory and close the file.
    // Idempotent (only writes on first call). Called automatically by ~Dtor.
    void close();

private:
    struct Entry {
        std::string name;
        std::uint32_t crc32;
        std::uint32_t size;     // both compressed and uncompressed (store mode)
        std::uint32_t offset;   // offset of local file header
    };

    // Low-level: write raw .npy header + data as one ZIP entry.
    void write_entry(const std::string& name,
                     const std::string& npy_header,
                     const void* data,
                     std::size_t data_size);

    void write_raw(const void* p, std::size_t n);
    void write_u16(std::uint16_t v);
    void write_u32(std::uint32_t v);

    std::FILE* fp_ = nullptr;
    std::vector<Entry> entries_;
    std::uint32_t pos_ = 0;
    bool closed_ = false;
};

}  // namespace calib
}  // namespace qmini
