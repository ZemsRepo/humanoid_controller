//
// Author: Unitree Robotics
// Organization: Unitree Robotics
// Source: https://github.com/unitreerobotics/unitree_rl_gym
//

#include "humanoid_controller/tasks/unitree_locomotion/Policy.h"

#include <onnxruntime/onnxruntime_cxx_api.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

namespace legged {

// ============================================================================
// Constructor
// ============================================================================

UnitreeLocomotionPolicy::UnitreeLocomotionPolicy(const std::string& modelPath) : modelPath_(modelPath) {
  // Create ONNX Runtime environment
  onnxEnvPtr_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "UnitreeLocomotionPolicy");

  // Create session options
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(1);
  sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  // Create session
  sessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPtr_, modelPath.c_str(), sessionOptions);
}

// ============================================================================
// Public Methods
// ============================================================================

void UnitreeLocomotionPolicy::init() {
  std::cout << "Initializing UnitreeLocomotionPolicy from: " << modelPath_ << std::endl;

  // Parse input/output shapes
  parseInputOutput();

  // Parse metadata from ONNX model
  parseMetadata();

  // Allocate tensor buffers
  inputTensor_ = tensor2d_t::Zero(1, static_cast<Eigen::Index>(numObservations_));
  outputTensor_ = tensor2d_t::Zero(1, static_cast<Eigen::Index>(numActions_));

  // Allocate LSTM state buffers if recurrent
  if (isRecurrent_) {
    hiddenState_ =
        tensor2d_t::Zero(static_cast<Eigen::Index>(rnnNumLayers_), static_cast<Eigen::Index>(rnnHiddenSize_));
    cellState_ = tensor2d_t::Zero(static_cast<Eigen::Index>(rnnNumLayers_), static_cast<Eigen::Index>(rnnHiddenSize_));
    std::cout << "  LSTM enabled: hidden_size=" << rnnHiddenSize_ << ", num_layers=" << rnnNumLayers_ << std::endl;
  }

  std::cout << "UnitreeLocomotionPolicy initialized successfully!" << std::endl;
  std::cout << "  Observation size: " << numObservations_ << std::endl;
  std::cout << "  Action size: " << numActions_ << std::endl;
  std::cout << "  Joint names: " << jointNames_.size() << std::endl;
  std::cout << "  Recurrent: " << (isRecurrent_ ? "yes" : "no") << std::endl;
}

void UnitreeLocomotionPolicy::reset() {
  outputTensor_.setZero();

  // Reset LSTM states
  if (isRecurrent_) {
    hiddenState_.setZero();
    cellState_.setZero();
  }
}

vector_t UnitreeLocomotionPolicy::forward(const vector_t& observations) {
  // Copy observations to input tensor
  for (Eigen::Index i = 0; i < observations.size(); ++i) {
    inputTensor_(0, i) = static_cast<tensor_element_t>(observations[i]);
  }

  // Run inference
  if (isRecurrent_) {
    runLSTM();
  } else {
    runMLP();
  }

  // Return action as Eigen vector
  return outputTensor_.row(0).cast<scalar_t>();
}

// ============================================================================
// Protected Methods - Metadata Parsing
// ============================================================================

void UnitreeLocomotionPolicy::parseInputOutput() {
  Ort::AllocatorWithDefaultOptions allocator;

  // Get input info
  size_t numInputs = sessionPtr_->GetInputCount();
  inputNamesRaw_.reserve(numInputs);
  inputNames_.reserve(numInputs);

  std::cout << "  ONNX Inputs:" << std::endl;
  for (size_t i = 0; i < numInputs; ++i) {
    inputNamesRaw_.push_back(sessionPtr_->GetInputNameAllocated(i, allocator));
    inputNames_.push_back(inputNamesRaw_.back().get());

    auto typeInfo = sessionPtr_->GetInputTypeInfo(i);
    auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
    auto shape = tensorInfo.GetShape();

    std::cout << "    [" << i << "] " << inputNames_[i] << " shape: [";
    for (size_t j = 0; j < shape.size(); ++j) {
      std::cout << shape[j];
      if (j < shape.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Extract observation size from "obs" input
    if (std::string(inputNames_[i]) == "obs" && shape.size() >= 2) {
      numObservations_ = static_cast<size_t>(shape[1]);
    }
  }

  // Get output info
  size_t numOutputs = sessionPtr_->GetOutputCount();
  outputNamesRaw_.reserve(numOutputs);
  outputNames_.reserve(numOutputs);

  std::cout << "  ONNX Outputs:" << std::endl;
  for (size_t i = 0; i < numOutputs; ++i) {
    outputNamesRaw_.push_back(sessionPtr_->GetOutputNameAllocated(i, allocator));
    outputNames_.push_back(outputNamesRaw_.back().get());

    auto typeInfo = sessionPtr_->GetOutputTypeInfo(i);
    auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
    auto shape = tensorInfo.GetShape();

    std::cout << "    [" << i << "] " << outputNames_[i] << " shape: [";
    for (size_t j = 0; j < shape.size(); ++j) {
      std::cout << shape[j];
      if (j < shape.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Extract action size from "actions" output
    if (std::string(outputNames_[i]) == "actions" && shape.size() >= 2) {
      numActions_ = static_cast<size_t>(shape[1]);
    }
  }

  // Check if model is recurrent (has hidden_state input)
  isRecurrent_ = (numInputs > 1);
}

void UnitreeLocomotionPolicy::parseMetadata() {
  std::cout << "Parsing ONNX metadata..." << std::endl;

  Ort::AllocatorWithDefaultOptions allocator;
  Ort::ModelMetadata modelMetadata = sessionPtr_->GetModelMetadata();

  // Get all metadata keys
  auto keysPtr = modelMetadata.GetCustomMetadataMapKeysAllocated(allocator);

  for (size_t i = 0; i < keysPtr.size(); ++i) {
    std::string key = keysPtr[i].get();
    auto valuePtr = modelMetadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator);
    std::string value = valuePtr.get();
    name2Metadata_[key] = value;
  }

  std::cout << "  Found " << name2Metadata_.size() << " metadata entries" << std::endl;

  // Parse joint_names
  try {
    auto it = name2Metadata_.find("joint_names");
    if (it != name2Metadata_.end()) {
      auto jointNamesJson = nlohmann::json::parse(it->second);
      jointNames_.clear();
      for (const auto& name : jointNamesJson) {
        jointNames_.push_back(name.get<std::string>());
      }
      std::cout << "  joint_names: " << jointNames_.size() << " joints" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse joint_names: " << e.what() << std::endl;
  }

  // Parse kp (stiffness)
  try {
    auto it = name2Metadata_.find("kp");
    if (it != name2Metadata_.end()) {
      auto kpJson = nlohmann::json::parse(it->second);
      std::vector<scalar_t> kpVec;
      for (const auto& val : kpJson) {
        kpVec.push_back(val.get<scalar_t>());
      }
      jointStiffness_ = Eigen::Map<vector_t>(kpVec.data(), static_cast<Eigen::Index>(kpVec.size()));
      std::cout << "  kp: " << jointStiffness_.transpose() << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse kp: " << e.what() << std::endl;
  }

  // Parse kd (damping)
  try {
    auto it = name2Metadata_.find("kd");
    if (it != name2Metadata_.end()) {
      auto kdJson = nlohmann::json::parse(it->second);
      std::vector<scalar_t> kdVec;
      for (const auto& val : kdJson) {
        kdVec.push_back(val.get<scalar_t>());
      }
      jointDamping_ = Eigen::Map<vector_t>(kdVec.data(), static_cast<Eigen::Index>(kdVec.size()));
      std::cout << "  kd: " << jointDamping_.transpose() << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse kd: " << e.what() << std::endl;
  }

  // Parse default_joint_pos
  try {
    auto it = name2Metadata_.find("default_joint_pos");
    if (it != name2Metadata_.end()) {
      auto posJson = nlohmann::json::parse(it->second);
      std::vector<scalar_t> posVec;
      for (const auto& val : posJson) {
        posVec.push_back(val.get<scalar_t>());
      }
      defaultJointPositions_ = Eigen::Map<vector_t>(posVec.data(), static_cast<Eigen::Index>(posVec.size()));
      std::cout << "  default_joint_pos: " << defaultJointPositions_.transpose() << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse default_joint_pos: " << e.what() << std::endl;
  }

  // Parse action_scale
  try {
    auto it = name2Metadata_.find("action_scale");
    if (it != name2Metadata_.end()) {
      scalar_t actionScaleValue = nlohmann::json::parse(it->second).get<scalar_t>();
      actionScale_ = vector_t::Constant(static_cast<Eigen::Index>(numActions_), actionScaleValue);
      std::cout << "  action_scale: " << actionScaleValue << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse action_scale: " << e.what() << std::endl;
    actionScale_ = vector_t::Constant(static_cast<Eigen::Index>(numActions_), 0.25);
  }

  // Parse observation scales
  try {
    auto it = name2Metadata_.find("ang_vel_scale");
    if (it != name2Metadata_.end()) {
      angVelScale_ = nlohmann::json::parse(it->second).get<scalar_t>();
    }
  } catch (...) {
  }

  try {
    auto it = name2Metadata_.find("dof_pos_scale");
    if (it != name2Metadata_.end()) {
      dofPosScale_ = nlohmann::json::parse(it->second).get<scalar_t>();
    }
  } catch (...) {
  }

  try {
    auto it = name2Metadata_.find("dof_vel_scale");
    if (it != name2Metadata_.end()) {
      dofVelScale_ = nlohmann::json::parse(it->second).get<scalar_t>();
    }
  } catch (...) {
  }

  try {
    auto it = name2Metadata_.find("cmd_scale");
    if (it != name2Metadata_.end()) {
      auto cmdScaleJson = nlohmann::json::parse(it->second);
      std::vector<scalar_t> cmdScaleVec;
      for (const auto& val : cmdScaleJson) {
        cmdScaleVec.push_back(val.get<scalar_t>());
      }
      cmdScale_ = Eigen::Map<vector_t>(cmdScaleVec.data(), static_cast<Eigen::Index>(cmdScaleVec.size()));
    }
  } catch (...) {
    cmdScale_ = vector_t::Constant(3, 1.0);
  }

  std::cout << "  ang_vel_scale: " << angVelScale_ << std::endl;
  std::cout << "  dof_pos_scale: " << dofPosScale_ << std::endl;
  std::cout << "  dof_vel_scale: " << dofVelScale_ << std::endl;
  std::cout << "  cmd_scale: " << cmdScale_.transpose() << std::endl;

  // Parse observation_names
  try {
    auto it = name2Metadata_.find("observation_names");
    if (it != name2Metadata_.end()) {
      auto obsNamesJson = nlohmann::json::parse(it->second);
      observationNames_.clear();
      for (const auto& name : obsNamesJson) {
        observationNames_.push_back(name.get<std::string>());
      }
      std::cout << "  observation_names: [";
      for (size_t i = 0; i < observationNames_.size(); ++i) {
        std::cout << observationNames_[i];
        if (i < observationNames_.size() - 1)
          std::cout << ", ";
      }
      std::cout << "]" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse observation_names: " << e.what() << std::endl;
  }

  // Set observation history lengths (unitree_rl_gym doesn't use history, LSTM handles temporal)
  observationHistoryLengths_.assign(observationNames_.size(), 1);

  // Parse LSTM configuration
  try {
    auto it = name2Metadata_.find("is_recurrent");
    if (it != name2Metadata_.end()) {
      isRecurrent_ = nlohmann::json::parse(it->second).get<bool>();
    }
  } catch (...) {
  }

  if (isRecurrent_) {
    try {
      auto it = name2Metadata_.find("rnn_type");
      if (it != name2Metadata_.end()) {
        rnnType_ = nlohmann::json::parse(it->second).get<std::string>();
      }
    } catch (...) {
    }

    try {
      auto it = name2Metadata_.find("rnn_hidden_size");
      if (it != name2Metadata_.end()) {
        rnnHiddenSize_ = nlohmann::json::parse(it->second).get<size_t>();
      }
    } catch (...) {
    }

    try {
      auto it = name2Metadata_.find("rnn_num_layers");
      if (it != name2Metadata_.end()) {
        rnnNumLayers_ = nlohmann::json::parse(it->second).get<size_t>();
      }
    } catch (...) {
    }
  }

  // Set command names (locomotion uses twist command)
  commandNames_.push_back("twist");
}

// ============================================================================
// ONNX Inference
// ============================================================================

void UnitreeLocomotionPolicy::runMLP() {
  auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // Create input tensor
  std::vector<int64_t> inputDims = {1, static_cast<int64_t>(numObservations_)};
  Ort::Value inputOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(
      memoryInfo, inputTensor_.data(), static_cast<size_t>(inputTensor_.size()), inputDims.data(), inputDims.size());

  // Create output tensor
  std::vector<int64_t> outputDims = {1, static_cast<int64_t>(numActions_)};
  Ort::Value outputOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, outputTensor_.data(),
                                                                          static_cast<size_t>(outputTensor_.size()),
                                                                          outputDims.data(), outputDims.size());

  // Run inference
  const char* inputNamesCStr[] = {inputNames_[0]};
  const char* outputNamesCStr[] = {outputNames_[0]};

  sessionPtr_->Run(Ort::RunOptions{nullptr}, inputNamesCStr, &inputOrtTensor, 1, outputNamesCStr, &outputOrtTensor, 1);
}

void UnitreeLocomotionPolicy::runLSTM() {
  auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // Input: obs [1, num_obs]
  std::vector<int64_t> obsDims = {1, static_cast<int64_t>(numObservations_)};
  Ort::Value obsOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(
      memoryInfo, inputTensor_.data(), static_cast<size_t>(inputTensor_.size()), obsDims.data(), obsDims.size());

  // Input: hidden_state [num_layers, 1, hidden_size]
  std::vector<int64_t> hiddenDims = {static_cast<int64_t>(rnnNumLayers_), 1, static_cast<int64_t>(rnnHiddenSize_)};
  Ort::Value hiddenOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(
      memoryInfo, hiddenState_.data(), static_cast<size_t>(hiddenState_.size()), hiddenDims.data(), hiddenDims.size());

  // Input: cell_state [num_layers, 1, hidden_size]
  std::vector<int64_t> cellDims = {static_cast<int64_t>(rnnNumLayers_), 1, static_cast<int64_t>(rnnHiddenSize_)};
  Ort::Value cellOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(
      memoryInfo, cellState_.data(), static_cast<size_t>(cellState_.size()), cellDims.data(), cellDims.size());

  // Output: actions [1, num_actions]
  std::vector<int64_t> actionDims = {1, static_cast<int64_t>(numActions_)};
  Ort::Value actionsOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, outputTensor_.data(),
                                                                           static_cast<size_t>(outputTensor_.size()),
                                                                           actionDims.data(), actionDims.size());

  // Output: hidden_state_out [num_layers, 1, hidden_size]
  tensor2d_t hiddenStateOut =
      tensor2d_t::Zero(static_cast<Eigen::Index>(rnnNumLayers_), static_cast<Eigen::Index>(rnnHiddenSize_));
  Ort::Value hiddenOutOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, hiddenStateOut.data(),
                                                                             static_cast<size_t>(hiddenStateOut.size()),
                                                                             hiddenDims.data(), hiddenDims.size());

  // Output: cell_state_out [num_layers, 1, hidden_size]
  tensor2d_t cellStateOut =
      tensor2d_t::Zero(static_cast<Eigen::Index>(rnnNumLayers_), static_cast<Eigen::Index>(rnnHiddenSize_));
  Ort::Value cellOutOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(
      memoryInfo, cellStateOut.data(), static_cast<size_t>(cellStateOut.size()), cellDims.data(), cellDims.size());

  // Prepare input/output arrays
  std::array<Ort::Value, 3> inputTensors{std::move(obsOrtTensor), std::move(hiddenOrtTensor), std::move(cellOrtTensor)};
  std::array<Ort::Value, 3> outputTensors{std::move(actionsOrtTensor), std::move(hiddenOutOrtTensor),
                                          std::move(cellOutOrtTensor)};

  const char* inputNamesCStr[] = {"obs", "hidden_state", "cell_state"};
  const char* outputNamesCStr[] = {"actions", "hidden_state_out", "cell_state_out"};

  // Run inference
  sessionPtr_->Run(Ort::RunOptions{nullptr}, inputNamesCStr, inputTensors.data(), 3, outputNamesCStr,
                   outputTensors.data(), 3);

  // Update hidden/cell states for next step
  hiddenState_ = hiddenStateOut;
  cellState_ = cellStateOut;
}

}  // namespace legged
