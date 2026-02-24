//
// Author: Amazon FAR
// Organization: Amazon FAR
// Source: https://github.com/amazon-far/holosoma
//

#include "humanoid_controller/tasks/holosoma_locomotion/Policy.h"

#include <onnxruntime/onnxruntime_cxx_api.h>

#include <algorithm>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace legged {

// ============================================================================
// Constructor
// ============================================================================

HolosomaLocomotionPolicy::HolosomaLocomotionPolicy(const std::string& modelPath) : modelPath_(modelPath) {
  // Create ONNX Runtime environment
  onnxEnvPtr_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HolosomaLocomotionPolicy");

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

void HolosomaLocomotionPolicy::init() {
  std::cout << "Initializing HolosomaLocomotionPolicy from: " << modelPath_ << std::endl;

  // Parse input/output shapes
  parseInputOutput();

  // Parse metadata from ONNX model
  parseMetadata();

  // Parse experiment config for action_scale and default_joint_positions
  parseExperimentConfig();

  // Allocate tensor buffers
  inputTensor_ = tensor2d_t::Zero(1, inputShape_[1]);
  outputTensor_ = tensor2d_t::Zero(1, outputShape_[1]);

  std::cout << "HolosomaLocomotionPolicy initialized successfully!" << std::endl;
  std::cout << "  Observation size: " << getObservationSize() << std::endl;
  std::cout << "  Action size: " << getActionSize() << std::endl;
  std::cout << "  Joint names: " << jointNames_.size() << std::endl;
}

void HolosomaLocomotionPolicy::reset() {
  outputTensor_ = tensor2d_t::Zero(1, outputShape_[1]);
}

vector_t HolosomaLocomotionPolicy::forward(const vector_t& observations) {
  // ObservationManager already handles history stacking, so observations
  // contains the full observation vector (with history if configured).
  // Just copy to input tensor and run inference.
  for (Eigen::Index i = 0; i < observations.size(); ++i) {
    inputTensor_(0, i) = static_cast<tensor_element_t>(observations[i]);
  }

  // Run inference
  run();

  // Return action as Eigen vector
  return outputTensor_.row(0).cast<scalar_t>();
}

// ============================================================================
// Protected Methods - Metadata Parsing
// ============================================================================

std::string HolosomaLocomotionPolicy::getMetadataStr(const std::string& key) const {
  auto it = name2Metadata_.find(key);
  if (it == name2Metadata_.end()) {
    throw std::runtime_error("HolosomaLocomotionPolicy: '" + key +
                             "' not found in model metadata. Please check the model.");
  }
  return it->second;
}

void HolosomaLocomotionPolicy::parseMetadata() {
  std::cout << "Parsing ONNX metadata (holosoma format)..." << std::endl;

  // Load ONNX model to extract metadata
  Ort::AllocatorWithDefaultOptions allocator;

  // Get model metadata
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

  // Parse dof_names (JSON array)
  try {
    std::string dofNamesJson = getMetadataStr("dof_names");
    auto dofNames = nlohmann::json::parse(dofNamesJson);
    jointNames_.clear();
    for (const auto& name : dofNames) {
      jointNames_.push_back(name.get<std::string>());
    }
    std::cout << "  dof_names: [";
    for (size_t i = 0; i < jointNames_.size(); ++i) {
      std::cout << jointNames_[i];
      if (i < jointNames_.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse dof_names: " << e.what() << std::endl;
  }

  // Parse kp (JSON array)
  try {
    std::string kpJson = getMetadataStr("kp");
    auto kpArray = nlohmann::json::parse(kpJson);
    std::vector<scalar_t> kpVec;
    for (const auto& val : kpArray) {
      kpVec.push_back(val.get<scalar_t>());
    }
    jointStiffness_ = Eigen::Map<vector_t>(kpVec.data(), static_cast<Eigen::Index>(kpVec.size()));
    std::cout << "  kp: " << jointStiffness_.transpose() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse kp: " << e.what() << std::endl;
  }

  // Parse kd (JSON array)
  try {
    std::string kdJson = getMetadataStr("kd");
    auto kdArray = nlohmann::json::parse(kdJson);
    std::vector<scalar_t> kdVec;
    for (const auto& val : kdArray) {
      kdVec.push_back(val.get<scalar_t>());
    }
    jointDamping_ = Eigen::Map<vector_t>(kdVec.data(), static_cast<Eigen::Index>(kdVec.size()));
    std::cout << "  kd: " << jointDamping_.transpose() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse kd: " << e.what() << std::endl;
  }
}

void HolosomaLocomotionPolicy::parseExperimentConfig() {
  std::cout << "Parsing experiment_config for action_scale, default_joint_positions, and observation_names..."
            << std::endl;

  try {
    std::string configJson = getMetadataStr("experiment_config");
    auto config = nlohmann::json::parse(configJson);

    // Extract action_scale from robot.control.action_scale
    scalar_t actionScaleValue = 1.0;
    if (config.contains("robot") && config["robot"].contains("control") &&
        config["robot"]["control"].contains("action_scale")) {
      actionScaleValue = config["robot"]["control"]["action_scale"].get<scalar_t>();
    }
    // Create action scale vector (same scale for all joints)
    actionScale_ = vector_t::Constant(static_cast<Eigen::Index>(jointNames_.size()), actionScaleValue);
    std::cout << "  action_scale: " << actionScaleValue << std::endl;

    // Extract default_joint_angles from robot.init_state.default_joint_angles
    if (config.contains("robot") && config["robot"].contains("init_state") &&
        config["robot"]["init_state"].contains("default_joint_angles")) {
      auto defaultAngles = config["robot"]["init_state"]["default_joint_angles"];

      // default_joint_angles is a dict mapping joint_name -> angle
      // We need to extract values in the order of jointNames_
      std::vector<scalar_t> defaultPosVec;
      for (const auto& jointName : jointNames_) {
        if (defaultAngles.contains(jointName)) {
          defaultPosVec.push_back(defaultAngles[jointName].get<scalar_t>());
        } else {
          std::cerr << "Warning: No default angle for joint " << jointName << ", using 0.0" << std::endl;
          defaultPosVec.push_back(0.0);
        }
      }
      defaultJointPositions_ =
          Eigen::Map<vector_t>(defaultPosVec.data(), static_cast<Eigen::Index>(defaultPosVec.size()));
      std::cout << "  default_joint_positions: " << defaultJointPositions_.transpose() << std::endl;
    } else {
      std::cerr << "Warning: default_joint_angles not found in experiment_config" << std::endl;
      defaultJointPositions_ = vector_t::Zero(static_cast<Eigen::Index>(jointNames_.size()));
    }

    // Extract observation_names from observation.groups.actor_obs.terms
    observationNames_.clear();
    observationHistoryLengths_.clear();
    size_t historyLength = 1;  // Default to no history

    if (config.contains("observation") && config["observation"].contains("groups") &&
        config["observation"]["groups"].contains("actor_obs")) {
      auto actorObsGroup = config["observation"]["groups"]["actor_obs"];

      // Get history_length for this group (default 1)
      if (actorObsGroup.contains("history_length")) {
        historyLength = actorObsGroup["history_length"].get<size_t>();
      }

      if (actorObsGroup.contains("terms")) {
        auto terms = actorObsGroup["terms"];
        // Terms is a dict, iterate over keys (term names)
        for (auto it = terms.begin(); it != terms.end(); ++it) {
          observationNames_.push_back(it.key());
          observationHistoryLengths_.push_back(historyLength);
        }
        // Sort observation names to match Python's sorted() behavior
        // Need to sort both vectors together
        std::vector<std::pair<std::string, size_t>> nameHistoryPairs;
        for (size_t i = 0; i < observationNames_.size(); ++i) {
          nameHistoryPairs.emplace_back(observationNames_[i], observationHistoryLengths_[i]);
        }
        std::sort(nameHistoryPairs.begin(), nameHistoryPairs.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        observationNames_.clear();
        observationHistoryLengths_.clear();
        for (const auto& pair : nameHistoryPairs) {
          observationNames_.push_back(pair.first);
          observationHistoryLengths_.push_back(pair.second);
        }
      }

      std::cout << "  observation_names: [";
      for (size_t i = 0; i < observationNames_.size(); ++i) {
        std::cout << observationNames_[i];
        if (i < observationNames_.size() - 1)
          std::cout << ", ";
      }
      std::cout << "]" << std::endl;
      std::cout << "  history_length: " << historyLength << std::endl;
    } else {
      std::cerr << "Warning: observation.groups.actor_obs not found in experiment_config" << std::endl;
    }

    // Extract command_names from command.terms
    commandNames_.clear();
    if (config.contains("command") && config["command"].contains("terms")) {
      auto terms = config["command"]["terms"];
      for (auto it = terms.begin(); it != terms.end(); ++it) {
        commandNames_.push_back(it.key());
      }
      std::cout << "  command_names: [";
      for (size_t i = 0; i < commandNames_.size(); ++i) {
        std::cout << commandNames_[i];
        if (i < commandNames_.size() - 1)
          std::cout << ", ";
      }
      std::cout << "]" << std::endl;
    } else {
      // Default command name for locomotion
      commandNames_.push_back("twist");
      std::cout << "  command_names (default): [twist]" << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse experiment_config: " << e.what() << std::endl;
    // Set default values
    actionScale_ = vector_t::Constant(static_cast<Eigen::Index>(jointNames_.size()), 0.25);
    defaultJointPositions_ = vector_t::Zero(static_cast<Eigen::Index>(jointNames_.size()));
  }
}

void HolosomaLocomotionPolicy::parseInputOutput() {
  Ort::AllocatorWithDefaultOptions allocator;

  // Get input info
  size_t numInputs = sessionPtr_->GetInputCount();
  inputNamesRaw_.reserve(numInputs);
  inputNames_.reserve(numInputs);

  for (size_t i = 0; i < numInputs; ++i) {
    inputNamesRaw_.push_back(sessionPtr_->GetInputNameAllocated(i, allocator));
    inputNames_.push_back(inputNamesRaw_.back().get());

    // Get input shape (for actor_obs)
    auto typeInfo = sessionPtr_->GetInputTypeInfo(i);
    auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
    inputShape_ = tensorInfo.GetShape();

    std::cout << "  Input[" << i << "]: " << inputNames_[i] << " shape: [";
    for (size_t j = 0; j < inputShape_.size(); ++j) {
      std::cout << inputShape_[j];
      if (j < inputShape_.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }

  // Get output info
  size_t numOutputs = sessionPtr_->GetOutputCount();
  outputNamesRaw_.reserve(numOutputs);
  outputNames_.reserve(numOutputs);

  for (size_t i = 0; i < numOutputs; ++i) {
    outputNamesRaw_.push_back(sessionPtr_->GetOutputNameAllocated(i, allocator));
    outputNames_.push_back(outputNamesRaw_.back().get());

    // Get output shape (for action)
    auto typeInfo = sessionPtr_->GetOutputTypeInfo(i);
    auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();
    outputShape_ = tensorInfo.GetShape();

    std::cout << "  Output[" << i << "]: " << outputNames_[i] << " shape: [";
    for (size_t j = 0; j < outputShape_.size(); ++j) {
      std::cout << outputShape_[j];
      if (j < outputShape_.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }
}

void HolosomaLocomotionPolicy::run() {
  // Create memory info
  auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // Create input tensor
  std::vector<int64_t> inputDims = {1, inputShape_[1]};
  Ort::Value inputOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(
      memoryInfo, inputTensor_.data(), static_cast<size_t>(inputTensor_.size()), inputDims.data(), inputDims.size());

  // Create output tensor
  std::vector<int64_t> outputDims = {1, outputShape_[1]};
  Ort::Value outputOrtTensor = Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, outputTensor_.data(),
                                                                          static_cast<size_t>(outputTensor_.size()),
                                                                          outputDims.data(), outputDims.size());

  // Run inference
  const char* inputNamesCStr[] = {inputNames_[0]};
  const char* outputNamesCStr[] = {outputNames_[0]};

  sessionPtr_->Run(Ort::RunOptions{nullptr}, inputNamesCStr, &inputOrtTensor, 1, outputNamesCStr, &outputOrtTensor, 1);
}
}  // namespace legged
