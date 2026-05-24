#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

namespace qmini {

// Inference interface so the control loop is testable without onnxruntime.
// Two implementations:
//   - OnnxPolicy:     wraps Ort::Session, defined only when WITH_ONNX is on
//   - IdentityPolicy: returns zeros of the right shape; useful for sim and
//                     for unit tests that exercise the control loop without
//                     needing a trained policy
class IPolicy {
public:
    virtual ~IPolicy() = default;
    virtual int input_dim()  const = 0;   // per-step obs dim
    virtual int output_dim() const = 0;   // action dim
    virtual int stack_dim()  const = 0;   // history frames
    // obs_flat has length input_dim * stack_dim; returns output_dim values.
    // Values are clamped to [-1, 1] by the implementation.
    virtual Eigen::VectorXf infer(const Eigen::VectorXf& obs_flat) = 0;
};

std::unique_ptr<IPolicy> make_identity_policy(
    int input_dim, int stack_dim, int output_dim);

// Defined only when WITH_ONNX is true. Throws std::runtime_error if the
// file is missing or its input/output shapes disagree with the given dims.
std::unique_ptr<IPolicy> make_onnx_policy(
    const std::string& path,
    int input_dim, int stack_dim, int output_dim);

}  // namespace qmini
