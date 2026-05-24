#include "user/policy.h"

#include <stdexcept>
#include <vector>

#include "onnx/onnxruntime_cxx_api.h"

namespace qmini {
namespace {

class OnnxPolicy : public IPolicy {
public:
    OnnxPolicy(const std::string& path, int in, int stack, int out)
        : input_dim_(in), stack_dim_(stack), output_dim_(out),
          env_(ORT_LOGGING_LEVEL_WARNING, "qmini") {
        Ort::SessionOptions opts;
        session_ = std::make_unique<Ort::Session>(env_, path.c_str(), opts);
        flat_size_ = static_cast<size_t>(in) * static_cast<size_t>(stack);
        input_shape_ = {1, static_cast<int64_t>(flat_size_)};
    }
    int input_dim()  const override { return input_dim_; }
    int output_dim() const override { return output_dim_; }
    int stack_dim()  const override { return stack_dim_; }
    Eigen::VectorXf infer(const Eigen::VectorXf& obs_flat) override {
        if (static_cast<size_t>(obs_flat.size()) != flat_size_) {
            throw std::runtime_error("OnnxPolicy: obs size mismatch");
        }
        std::vector<float> buf(obs_flat.data(),
                               obs_flat.data() + obs_flat.size());
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,
                                              OrtMemTypeDefault);
        auto in_tensor = Ort::Value::CreateTensor<float>(
            mem, buf.data(), buf.size(),
            input_shape_.data(), input_shape_.size());
        const char* in_names[]  = {"input"};
        const char* out_names[] = {"output"};
        auto out = session_->Run(Ort::RunOptions{nullptr}, in_names,
                                 &in_tensor, 1, out_names, 1);
        float* p = out[0].GetTensorMutableData<float>();
        Eigen::VectorXf r(output_dim_);
        for (int i = 0; i < output_dim_; ++i) {
            float v = p[i];
            if (v >  1.f) v =  1.f;
            if (v < -1.f) v = -1.f;
            r[i] = v;
        }
        return r;
    }
private:
    int input_dim_, stack_dim_, output_dim_;
    size_t flat_size_;
    std::vector<int64_t> input_shape_;
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
};

}  // namespace

std::unique_ptr<IPolicy> make_onnx_policy(
    const std::string& path, int in, int stack, int out) {
    return std::make_unique<OnnxPolicy>(path, in, stack, out);
}

}  // namespace qmini
