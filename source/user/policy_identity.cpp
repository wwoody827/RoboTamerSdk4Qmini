#include "user/policy.h"

namespace qmini {
namespace {

class IdentityPolicy : public IPolicy {
public:
    IdentityPolicy(int in, int stack, int out)
        : input_dim_(in), stack_dim_(stack), output_dim_(out) {}
    int input_dim()  const override { return input_dim_; }
    int output_dim() const override { return output_dim_; }
    int stack_dim()  const override { return stack_dim_; }
    Eigen::VectorXf infer(const Eigen::VectorXf&) override {
        return Eigen::VectorXf::Zero(output_dim_);
    }
private:
    int input_dim_, stack_dim_, output_dim_;
};

}  // namespace

std::unique_ptr<IPolicy> make_identity_policy(int in, int stack, int out) {
    return std::make_unique<IdentityPolicy>(in, stack, out);
}

}  // namespace qmini
