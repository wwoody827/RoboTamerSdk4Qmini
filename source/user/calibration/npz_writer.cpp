#include "user/calibration/npz_writer.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace qmini {
namespace calib {
namespace {

// CRC32 using IEEE polynomial 0xEDB88320 (reflected, what ZIP expects).
std::uint32_t crc32_update(std::uint32_t crc, const void* buf, std::size_t n) {
    static std::uint32_t table[256];
    static bool inited = false;
    if (!inited) {
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        inited = true;
    }
    const std::uint8_t* p = static_cast<const std::uint8_t*>(buf);
    crc ^= 0xFFFFFFFFu;
    for (std::size_t i = 0; i < n; ++i) {
        crc = table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

// Build a properly-padded NPY header bytes. `descr_dict` is the part inside
// the braces, e.g. "'descr': '<f4', 'fortran_order': False, 'shape': (1800, 10), ".
std::string make_npy_header(const std::string& descr_dict) {
    std::string magic = "\x93NUMPY";
    // version 1.0
    std::string out = magic + std::string("\x01\x00", 2);
    std::string body = "{" + descr_dict + "}";
    // pad with spaces, terminate with '\n', so total (magic+ver+lenfield+body) % 64 == 0.
    // header preamble is 10 bytes (6 magic + 2 version + 2 length).
    std::size_t base = 10 + body.size() + 1;  // +1 for trailing '\n'
    std::size_t pad = (64 - (base % 64)) % 64;
    body.append(pad, ' ');
    body.push_back('\n');
    std::uint16_t header_len = static_cast<std::uint16_t>(body.size());
    char len_le[2] = {static_cast<char>(header_len & 0xFF),
                     static_cast<char>((header_len >> 8) & 0xFF)};
    out.append(len_le, 2);
    out.append(body);
    return out;
}

}  // namespace

NpzWriter::NpzWriter(const std::string& path) {
    fp_ = std::fopen(path.c_str(), "wb");
    if (!fp_) {
        throw std::runtime_error("NpzWriter: cannot open " + path);
    }
}

NpzWriter::~NpzWriter() { close(); }

void NpzWriter::write_raw(const void* p, std::size_t n) {
    if (n == 0) return;
    if (std::fwrite(p, 1, n, fp_) != n) {
        throw std::runtime_error("NpzWriter: short write");
    }
    pos_ += static_cast<std::uint32_t>(n);
}
void NpzWriter::write_u16(std::uint16_t v) {
    std::uint8_t b[2] = {static_cast<std::uint8_t>(v & 0xFF),
                         static_cast<std::uint8_t>((v >> 8) & 0xFF)};
    write_raw(b, 2);
}
void NpzWriter::write_u32(std::uint32_t v) {
    std::uint8_t b[4] = {static_cast<std::uint8_t>(v & 0xFF),
                         static_cast<std::uint8_t>((v >> 8) & 0xFF),
                         static_cast<std::uint8_t>((v >> 16) & 0xFF),
                         static_cast<std::uint8_t>((v >> 24) & 0xFF)};
    write_raw(b, 4);
}

void NpzWriter::write_entry(const std::string& name,
                            const std::string& npy_header,
                            const void* data,
                            std::size_t data_size) {
    // Inside the .npz, each file is named "<name>.npy".
    std::string fname = name + ".npy";
    Entry e;
    e.name = fname;
    e.offset = pos_;
    e.crc32 = 0;
    e.crc32 = crc32_update(e.crc32, npy_header.data(), npy_header.size());
    e.crc32 = crc32_update(e.crc32, data, data_size);
    e.size = static_cast<std::uint32_t>(npy_header.size() + data_size);

    // Local File Header (30 bytes + filename)
    write_u32(0x04034b50);          // signature
    write_u16(20);                  // version needed
    write_u16(0);                   // flags
    write_u16(0);                   // compression = stored
    write_u16(0);                   // mod time
    write_u16(0x21);                // mod date (placeholder: 1980-01-01)
    write_u32(e.crc32);
    write_u32(e.size);              // compressed
    write_u32(e.size);              // uncompressed
    write_u16(static_cast<std::uint16_t>(fname.size()));
    write_u16(0);                   // extra len
    write_raw(fname.data(), fname.size());
    // File data: header then payload
    write_raw(npy_header.data(), npy_header.size());
    write_raw(data, data_size);

    entries_.push_back(e);
}

void NpzWriter::add_f32_2d(const std::string& name,
                           const float* data,
                           std::size_t rows,
                           std::size_t cols) {
    char shape[64];
    std::snprintf(shape, sizeof(shape), "(%zu, %zu)", rows, cols);
    std::string descr = "'descr': '<f4', 'fortran_order': False, 'shape': ";
    descr += shape;
    descr += ", ";
    write_entry(name, make_npy_header(descr), data,
                rows * cols * sizeof(float));
}

void NpzWriter::add_f32_1d(const std::string& name,
                           const float* data,
                           std::size_t n) {
    char shape[64];
    std::snprintf(shape, sizeof(shape), "(%zu,)", n);
    std::string descr = "'descr': '<f4', 'fortran_order': False, 'shape': ";
    descr += shape;
    descr += ", ";
    write_entry(name, make_npy_header(descr), data, n * sizeof(float));
}

void NpzWriter::add_f64_1d(const std::string& name,
                           const double* data,
                           std::size_t n) {
    char shape[64];
    std::snprintf(shape, sizeof(shape), "(%zu,)", n);
    std::string descr = "'descr': '<f8', 'fortran_order': False, 'shape': ";
    descr += shape;
    descr += ", ";
    write_entry(name, make_npy_header(descr), data, n * sizeof(double));
}

void NpzWriter::add_i64_scalar(const std::string& name, std::int64_t v) {
    std::string descr =
        "'descr': '<i8', 'fortran_order': False, 'shape': (), ";
    write_entry(name, make_npy_header(descr), &v, sizeof(v));
}

void NpzWriter::add_f64_scalar(const std::string& name, double v) {
    std::string descr =
        "'descr': '<f8', 'fortran_order': False, 'shape': (), ";
    write_entry(name, make_npy_header(descr), &v, sizeof(v));
}

void NpzWriter::add_string_scalar(const std::string& name,
                                  const std::string& value) {
    // Store as numpy <UN (UCS-4 little-endian) with shape ().
    // N = number of codepoints. Treat input as ASCII (good enough for trial
    // labels). One codepoint = 4 bytes.
    std::size_t n = value.size();
    std::vector<std::uint32_t> ucs(n);
    for (std::size_t i = 0; i < n; ++i) ucs[i] = static_cast<std::uint8_t>(value[i]);
    char descr[96];
    std::snprintf(descr, sizeof(descr),
                  "'descr': '<U%zu', 'fortran_order': False, 'shape': (), ", n);
    write_entry(name, make_npy_header(descr), ucs.data(), n * sizeof(std::uint32_t));
}

void NpzWriter::close() {
    if (closed_ || !fp_) return;
    // Central directory
    std::uint32_t cd_offset = pos_;
    for (const auto& e : entries_) {
        write_u32(0x02014b50);      // CDH signature
        write_u16(20);              // version made by
        write_u16(20);              // version to extract
        write_u16(0);               // flags
        write_u16(0);               // compression = stored
        write_u16(0);               // mod time
        write_u16(0x21);            // mod date
        write_u32(e.crc32);
        write_u32(e.size);
        write_u32(e.size);
        write_u16(static_cast<std::uint16_t>(e.name.size()));
        write_u16(0);               // extra len
        write_u16(0);               // comment len
        write_u16(0);               // disk number start
        write_u16(0);               // internal attrs
        write_u32(0);               // external attrs
        write_u32(e.offset);        // LFH offset
        write_raw(e.name.data(), e.name.size());
    }
    std::uint32_t cd_size = pos_ - cd_offset;
    // EOCD
    write_u32(0x06054b50);
    write_u16(0); write_u16(0);
    write_u16(static_cast<std::uint16_t>(entries_.size()));
    write_u16(static_cast<std::uint16_t>(entries_.size()));
    write_u32(cd_size);
    write_u32(cd_offset);
    write_u16(0);                   // comment len
    std::fclose(fp_);
    fp_ = nullptr;
    closed_ = true;
}

}  // namespace calib
}  // namespace qmini
