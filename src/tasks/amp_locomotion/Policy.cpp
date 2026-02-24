//
// Author: Zitong Bai
// Organization: BUAA
// Source: https://github.com/zitongbai/legged_lab
//

#include "humanoid_controller/tasks/amp_locomotion/Policy.h"

#include <onnxruntime/onnxruntime_cxx_api.h>

#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace legged {

// ============================================================================
// Constructor
// ============================================================================

AmpLocomotionPolicy::AmpLocomotionPolicy(const std::string& modelPath) : modelPath_(modelPath) {
  // Create ONNX Runtime environment
  onnxEnvPtr_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "AmpLocomotionPolicy");

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

void AmpLocomotionPolicy::init() {
  std::cout << "Initializing AmpLocomotionPolicy from: " << modelPath_ << std::endl;

  // Parse input/output shapes
  parseInputOutput();

  // Parse metadata from ONNX model
  parseMetadata();

  // Allocate tensor buffers
  inputTensor_ = tensor2d_t::Zero(1, inputShape_[1]);
  outputTensor_ = tensor2d_t::Zero(1, outputShape_[1]);

  std::cout << "AmpLocomotionPolicy initialized successfully!" << std::endl;
  std::cout << "  Observation size: " << getObservationSize() << std::endl;
  std::cout << "  Action size: " << getActionSize() << std::endl;
  std::cout << "  History length: " << historyLength_ << std::endl;
  std::cout << "  Joint names: " << jointNames_.size() << std::endl;
}

void AmpLocomotionPolicy::reset() {
  outputTensor_ = tensor2d_t::Zero(1, outputShape_[1]);
}

vector_t AmpLocomotionPolicy::forward(const vector_t& observations) {
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

std::string AmpLocomotionPolicy::getMetadataStr(const std::string& key) const {
  auto it = name2Metadata_.find(key);
  if (it == name2Metadata_.end()) {
    throw std::runtime_error("AmpLocomotionPolicy: '" + key + "' not found in model metadata. Please check the model.");
  }
  return it->second;
}

std::string AmpLocomotionPolicy::tryGetMetadataStr(const std::string& key) const {
  auto it = name2Metadata_.find(key);
  if (it == name2Metadata_.end()) {
    return "";
  }
  return it->second;
}

void AmpLocomotionPolicy::parseMetadata() {
  std::cout << "Parsing ONNX metadata (AMP format)..." << std::endl;

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

  // Parse joint_names (JSON array)
  try {
    std::string jointNamesJson = getMetadataStr("joint_names");
    auto jointNamesArray = nlohmann::json::parse(jointNamesJson);
    jointNames_.clear();
    for (const auto& name : jointNamesArray) {
      jointNames_.push_back(name.get<std::string>());
    }
    std::cout << "  joint_names: " << jointNames_.size() << " joints" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse joint_names: " << e.what() << std::endl;
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

  // Parse default_joint_pos (JSON array)
  try {
    std::string defaultPosJson = getMetadataStr("default_joint_pos");
    auto posArray = nlohmann::json::parse(defaultPosJson);
    std::vector<scalar_t> posVec;
    for (const auto& val : posArray) {
      posVec.push_back(val.get<scalar_t>());
    }
    defaultJointPositions_ = Eigen::Map<vector_t>(posVec.data(), static_cast<Eigen::Index>(posVec.size()));
    std::cout << "  default_joint_pos: " << defaultJointPositions_.transpose() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse default_joint_pos: " << e.what() << std::endl;
    defaultJointPositions_ = vector_t::Zero(static_cast<Eigen::Index>(jointNames_.size()));
  }

  // Parse action_scale (can be scalar or array)
  try {
    std::string actionScaleJson = getMetadataStr("action_scale");
    auto actionScaleValue = nlohmann::json::parse(actionScaleJson);
    if (actionScaleValue.is_number()) {
      scalar_t scale = actionScaleValue.get<scalar_t>();
      actionScale_ = vector_t::Constant(static_cast<Eigen::Index>(jointNames_.size()), scale);
      std::cout << "  action_scale: " << scale << std::endl;
    } else if (actionScaleValue.is_array()) {
      std::vector<scalar_t> scaleVec;
      for (const auto& val : actionScaleValue) {
        scaleVec.push_back(val.get<scalar_t>());
      }
      actionScale_ = Eigen::Map<vector_t>(scaleVec.data(), static_cast<Eigen::Index>(scaleVec.size()));
      std::cout << "  action_scale: " << actionScale_.transpose() << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse action_scale: " << e.what() << std::endl;
    actionScale_ = vector_t::Constant(static_cast<Eigen::Index>(jointNames_.size()), 0.25);
  }

  // Parse observation_names (JSON array)
  try {
    std::string obsNamesJson = getMetadataStr("observation_names");
    auto obsNamesArray = nlohmann::json::parse(obsNamesJson);
    observationNames_.clear();
    for (const auto& name : obsNamesArray) {
      observationNames_.push_back(name.get<std::string>());
    }
    std::cout << "  observation_names: [";
    for (size_t i = 0; i < observationNames_.size(); ++i) {
      std::cout << observationNames_[i];
      if (i < observationNames_.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse observation_names: " << e.what() << std::endl;
  }

  // Parse observation_history_lengths (JSON object: name -> length)
  try {
    std::string historyJson = tryGetMetadataStr("observation_history_lengths");
    if (!historyJson.empty()) {
      auto historyObj = nlohmann::json::parse(historyJson);
      // Get history length from first observation term (assuming uniform)
      if (!historyObj.empty()) {
        historyLength_ = historyObj.begin().value().get<size_t>();
      }
      observationHistoryLengths_.clear();
      for (const auto& name : observationNames_) {
        if (historyObj.contains(name)) {
          observationHistoryLengths_.push_back(historyObj[name].get<size_t>());
        } else {
          observationHistoryLengths_.push_back(1);
        }
      }
      std::cout << "  history_length: " << historyLength_ << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Warning: Failed to parse observation_history_lengths: " << e.what() << std::endl;
  }

  // Parse command_names (JSON array) - optional
  try {
    std::string cmdNamesJson = tryGetMetadataStr("command_names");
    if (!cmdNamesJson.empty()) {
      auto cmdNamesArray = nlohmann::json::parse(cmdNamesJson);
      commandNames_.clear();
      for (const auto& name : cmdNamesArray) {
        commandNames_.push_back(name.get<std::string>());
      }
    } else {
      // Default command for locomotion
      commandNames_.push_back("twist");
    }
    std::cout << "  command_names: [";
    for (size_t i = 0; i < commandNames_.size(); ++i) {
      std::cout << commandNames_[i];
      if (i < commandNames_.size() - 1)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  } catch (const std::exception& e) {
    commandNames_.push_back("twist");
    std::cerr << "Warning: Failed to parse command_names: " << e.what() << std::endl;
  }
}

void AmpLocomotionPolicy::parseInputOutput() {
  Ort::AllocatorWithDefaultOptions allocator;

  // Get input info
  size_t numInputs = sessionPtr_->GetInputCount();
  inputNamesRaw_.reserve(numInputs);
  inputNames_.reserve(numInputs);

  for (size_t i = 0; i < numInputs; ++i) {
    inputNamesRaw_.push_back(sessionPtr_->GetInputNameAllocated(i, allocator));
    inputNames_.push_back(inputNamesRaw_.back().get());

    // Get input shape
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

    // Get output shape
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

void AmpLocomotionPolicy::run() {
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
