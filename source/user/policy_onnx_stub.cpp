// Stub used when WITH_ONNX=OFF. Keeps the symbol defined so callers don't
// need ifdefs; throws if anyone actually asks for the ONNX backend.

#include <stdexcept>

#include "user/policy.h"

namespace qmini {

std::unique_ptr<IPolicy> make_onnx_policy(
    const std::string&, int, int, int) {
    throw std::runtime_error(
        "ONNX policy requested but this build was configured with "
        "WITH_ONNX=OFF. Rebuild with -DWITH_ONNX=ON or pass --no-onnx.");
}

}  // namespace qmini
