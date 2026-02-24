//
// Author: Amazon FAR
// Organization: Amazon FAR
// Source: https://github.com/amazon-far/holosoma
//

#pragma once

#include <legged_rl_controllers/Policy.h>
#include <onnxruntime/onnxruntime_cxx_api.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace legged {

class HolosomaLocomotionPolicy : public Policy {
 public:
  using SharedPtr = std::shared_ptr<HolosomaLocomotionPolicy>;
  using tensor_element_t = float;
  using tensor2d_t = Eigen::Matrix<tensor_element_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  /**
   * @brief Construct a new HolosomaLocomotionPolicy
   * @param modelPath Path to the ONNX model file (holosoma format)
   */
  explicit HolosomaLocomotionPolicy(const std::string& modelPath);

  /**
   * @brief Get the observation input size
   */
  size_t getObservationSize() const override {
    return inputShape_[1];
  }

  /**
   * @brief Get the action output size
   */
  size_t getActionSize() const override {
    return outputShape_[1];
  }

  /**
   * @brief Initialize the policy (parse metadata and setup tensors)
   */
  void init() override;

  /**
   * @brief Reset the policy state (zero out action buffer)
   */
  void reset() override;

  /**
   * @brief Get the last computed action
   */
  vector_t getLastAction() override {
    return outputTensor_.row(0).cast<scalar_t>();
  }

  /**
   * @brief Run forward inference
   * @param observations Input observation vector
   * @return Action vector
   */
  vector_t forward(const vector_t& observations) override;

 protected:
  /**
   * @brief Get metadata string from ONNX model
   * @param key Metadata key name
   * @return Metadata value as string
   * @throws std::runtime_error if key not found
   */
  std::string getMetadataStr(const std::string& key) const;

  /**
   * @brief Parse all metadata from ONNX model (dof_names, kp, kd, etc.)
   */
  void parseMetadata();

  /**
   * @brief Parse experiment_config JSON to extract action_scale and default_joint_positions
   */
  void parseExperimentConfig();

  /**
   * @brief Parse ONNX input/output shapes and names
   */
  void parseInputOutput();

  /**
   * @brief Run ONNX inference
   */
  void run();

  // ONNX Runtime members
  std::shared_ptr<Ort::Env> onnxEnvPtr_;
  std::unique_ptr<Ort::Session> sessionPtr_;
  std::map<std::string, std::string> name2Metadata_;

  // Input/output info
  std::vector<Ort::AllocatedStringPtr> inputNamesRaw_;
  std::vector<Ort::AllocatedStringPtr> outputNamesRaw_;
  std::vector<const char*> inputNames_;
  std::vector<const char*> outputNames_;
  std::vector<int64_t> inputShape_;
  std::vector<int64_t> outputShape_;

  // Tensor buffers
  tensor2d_t inputTensor_;
  tensor2d_t outputTensor_;

  // Model path
  std::string modelPath_;
};

}  // namespace legged
