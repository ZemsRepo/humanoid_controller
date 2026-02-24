//
// Author: Unitree Robotics
// Organization: Unitree Robotics
// Source: https://github.com/unitreerobotics/unitree_rl_gym
//

#pragma once

#include <legged_rl_controllers/Policy.h>
#include <onnxruntime/onnxruntime_cxx_api.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace legged {

class UnitreeLocomotionPolicy : public Policy {
 public:
  using SharedPtr = std::shared_ptr<UnitreeLocomotionPolicy>;
  using tensor_element_t = float;
  using tensor2d_t = Eigen::Matrix<tensor_element_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  /**
   * @brief Construct a new UnitreeLocomotionPolicy
   * @param modelPath Path to the ONNX model file
   */
  explicit UnitreeLocomotionPolicy(const std::string& modelPath);

  /**
   * @brief Get the observation input size
   */
  size_t getObservationSize() const override {
    return numObservations_;
  }

  /**
   * @brief Get the action output size
   */
  size_t getActionSize() const override {
    return numActions_;
  }

  /**
   * @brief Initialize the policy (parse metadata and setup tensors)
   */
  void init() override;

  /**
   * @brief Reset the policy state (zero out action buffer and LSTM states)
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

  // Getters for observation scales (used by observation terms)
  scalar_t getAngVelScale() const {
    return angVelScale_;
  }
  scalar_t getDofPosScale() const {
    return dofPosScale_;
  }
  scalar_t getDofVelScale() const {
    return dofVelScale_;
  }
  const vector_t& getCmdScale() const {
    return cmdScale_;
  }

  // LSTM configuration
  bool isRecurrent() const {
    return isRecurrent_;
  }
  size_t getRnnHiddenSize() const {
    return rnnHiddenSize_;
  }
  size_t getRnnNumLayers() const {
    return rnnNumLayers_;
  }

 protected:
  /**
   * @brief Parse ONNX input/output shapes and names
   */
  void parseInputOutput();

  /**
   * @brief Parse metadata from ONNX model
   */
  void parseMetadata();

  /**
   * @brief Run ONNX inference (MLP version)
   */
  void runMLP();

  /**
   * @brief Run ONNX inference (LSTM version)
   */
  void runLSTM();

  // ONNX Runtime members
  std::shared_ptr<Ort::Env> onnxEnvPtr_;
  std::unique_ptr<Ort::Session> sessionPtr_;
  std::map<std::string, std::string> name2Metadata_;

  // Input/output info
  std::vector<Ort::AllocatedStringPtr> inputNamesRaw_;
  std::vector<Ort::AllocatedStringPtr> outputNamesRaw_;
  std::vector<const char*> inputNames_;
  std::vector<const char*> outputNames_;

  // Model dimensions
  size_t numObservations_{0};
  size_t numActions_{0};

  // Observation tensor (input)
  tensor2d_t inputTensor_;
  // Action tensor (output)
  tensor2d_t outputTensor_;

  // LSTM state buffers [num_layers, hidden_size]
  tensor2d_t hiddenState_;
  tensor2d_t cellState_;

  // LSTM configuration from metadata
  bool isRecurrent_{false};
  std::string rnnType_{"lstm"};
  size_t rnnHiddenSize_{64};
  size_t rnnNumLayers_{1};

  // Observation scales from metadata
  scalar_t angVelScale_{0.25};
  scalar_t dofPosScale_{1.0};
  scalar_t dofVelScale_{0.05};
  vector_t cmdScale_;

  // Model path
  std::string modelPath_;
};

}  // namespace legged
