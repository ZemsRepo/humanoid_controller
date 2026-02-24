//
// Author: Zitong Bai
// Organization: BUAA
// Source: https://github.com/zitongbai/legged_lab
//

#pragma once

#include <legged_rl_controllers/Policy.h>
#include <onnxruntime/onnxruntime_cxx_api.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace legged {

/**
 * @brief AMP Locomotion Policy - Simple MLP policy for AMP-trained models
 *
 * This policy handles observation history through the ObservationManager's
 * built-in history buffer (historyLength_ field). The ONNX model expects
 * flattened observations with history already concatenated.
 */
class AmpLocomotionPolicy : public Policy {
 public:
  using SharedPtr = std::shared_ptr<AmpLocomotionPolicy>;
  using tensor_element_t = float;
  using tensor2d_t = Eigen::Matrix<tensor_element_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  /**
   * @brief Construct a new AmpLocomotionPolicy
   * @param modelPath Path to the ONNX model file
   */
  explicit AmpLocomotionPolicy(const std::string& modelPath);

  /**
   * @brief Get the observation input size (single frame, before history expansion)
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
   * @param observations Input observation vector (with history if configured)
   * @return Action vector
   */
  vector_t forward(const vector_t& observations) override;

  /**
   * @brief Get the observation history length from metadata
   */
  size_t getHistoryLength() const {
    return historyLength_;
  }

 protected:
  /**
   * @brief Get metadata string from ONNX model
   * @param key Metadata key name
   * @return Metadata value as string
   * @throws std::runtime_error if key not found
   */
  std::string getMetadataStr(const std::string& key) const;

  /**
   * @brief Try to get metadata string, return empty if not found
   */
  std::string tryGetMetadataStr(const std::string& key) const;

  /**
   * @brief Parse all metadata from ONNX model
   */
  void parseMetadata();

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

  // History length for observation stacking
  size_t historyLength_{5};

  // Model path
  std::string modelPath_;
};

}  // namespace legged
