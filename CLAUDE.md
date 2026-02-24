# CLAUDE.md - humanoid_controller Development Guide

This document provides instructions for AI assistants (Claude) and developers to understand and extend the humanoid_controller package.

## Project Overview

This is a ROS 2 Humble package implementing neural network-based controllers for humanoid robots using the `legged_control2` framework. Controllers run ONNX policy inference to generate joint commands.

## Architecture

```
humanoid_controller/
├── include/humanoid_controller/
│   ├── common.h                    # Shared utilities (quaternion helpers)
│   └── tasks/
│       └── <task_name>/
│           ├── Controller.h        # Main controller class
│           ├── Policy.h            # ONNX policy wrapper
│           ├── Command.h           # Command terms (reference inputs)
│           └── Observation.h       # Observation terms (state measurements)
├── src/tasks/<task_name>/          # Implementation files (.cpp)
├── config/<org>/<robot>/           # YAML configurations
├── launch/                         # ROS 2 launch files
└── humanoid_controller.xml         # Plugin registration
```

## Key Base Classes (from legged_control2 - closed source)

| Class | Header | Purpose |
|-------|--------|---------|
| `RlController` | `<legged_rl_controllers/RlController.h>` | Base controller with lifecycle, observation/command managers |
| `Policy` | `<legged_rl_controllers/Policy.h>` | Low-level ONNX session wrapper; used directly by most tasks |
| `OnnxPolicy` | `<legged_rl_controllers/OnnxPolicy.h>` | High-level ONNX wrapper with named I/O tensors and metadata parsing; extends `Policy` |
| `CommandTerm` | `<legged_rl_controllers/CommandManager.h>` | Base class for reference command providers |
| `ObservationTerm` | `<legged_rl_controllers/ObservationManager.h>` | Base class for state observations |
| `JointObservationTerm` | `<legged_rl_controllers/ObservationManager.h>` | Joint-specific observation base class |

## Common Types (from legged_model)

```cpp
#include <legged_model/common.h>

using scalar_t = double;
using vector_t = Eigen::VectorXd;
using vector3_t = Eigen::Vector3d;
using matrix3_t = Eigen::Matrix3d;
using quaternion_t = Eigen::Quaterniond;
```

---

## Creating a New Task

### Step 1: Create Directory Structure

```bash
mkdir -p include/humanoid_controller/tasks/<task_name>
mkdir -p src/tasks/<task_name>
```

### Step 2: Implement the Four Core Components

#### 2.1 Policy.h / Policy.cpp

Two base classes are available depending on the required level of control:

- **`OnnxPolicy`** (high-level): Named I/O tensors via `name2Index_` / `outputTensors_`, built-in `parseMetadata()`, `getLastAction()`. Used by `motion_tracking`.
- **`Policy`** (low-level): Direct ONNX Runtime session management. Used by `unitree_locomotion`, `holosoma_locomotion`, and `amp_locomotion` for custom input layouts, LSTM state, or observation history.

The example below uses the high-level `OnnxPolicy` approach (matches `motion_tracking`). For the low-level `Policy` approach, refer to the existing locomotion task implementations.

```cpp
// include/humanoid_controller/tasks/<task_name>/Policy.h
#pragma once

#include <legged_rl_controllers/OnnxPolicy.h>

namespace legged {

class MyTaskPolicy : public OnnxPolicy {
 public:
  using SharedPtr = std::shared_ptr<MyTaskPolicy>;

  MyTaskPolicy(const std::string& modelPath) : OnnxPolicy(modelPath) {}

  void reset() override;
  vector_t forward(const vector_t& observations) override;
  void parseMetadata() override;

  // Task-specific getters
  vector_t getCustomOutput() const { return customOutput_; }

 protected:
  vector_t customOutput_;
  // Parse from ONNX metadata
  std::string customParam_;
};

}  // namespace legged
```

```cpp
// src/tasks/<task_name>/Policy.cpp
#include "humanoid_controller/tasks/<task_name>/Policy.h"

namespace legged {

void MyTaskPolicy::reset() {
  OnnxPolicy::reset();
  // Initialize task-specific state
}

vector_t MyTaskPolicy::forward(const vector_t& observations) {
  OnnxPolicy::forward(observations);

  // Extract task-specific outputs from outputTensors_
  customOutput_ = outputTensors_[name2Index_.at("custom_output")].row(0).cast<scalar_t>();

  return getLastAction();
}

void MyTaskPolicy::parseMetadata() {
  OnnxPolicy::parseMetadata();
  customParam_ = getMetadataStr("custom_param");
}

}  // namespace legged
```

#### 2.2 Command.h / Command.cpp

Define command terms that provide reference inputs to the controller.

```cpp
// include/humanoid_controller/tasks/<task_name>/Command.h
#pragma once

#include <legged_rl_controllers/CommandManager.h>
#include "humanoid_controller/tasks/<task_name>/Policy.h"

namespace legged {

struct MyTaskCommandCfg {
  double param1;
  std::string param2;
};

class MyTaskCommandTerm : public CommandTerm {
 public:
  using SharedPtr = std::shared_ptr<MyTaskCommandTerm>;

  MyTaskCommandTerm(MyTaskCommandCfg cfg, MyTaskPolicy::SharedPtr policy)
      : cfg_(std::move(cfg)), policy_(std::move(policy)) {}

  vector_t getValue() override;
  void reset() override;

 protected:
  size_t getSize() const override { return 2 * model_->getNumJoints(); }

  MyTaskCommandCfg cfg_;
  MyTaskPolicy::SharedPtr policy_;
};

}  // namespace legged
```

```cpp
// src/tasks/<task_name>/Command.cpp
#include "humanoid_controller/tasks/<task_name>/Command.h"

namespace legged {

vector_t MyTaskCommandTerm::getValue() {
  // Return [joint_position, joint_velocity] or custom format
  return (vector_t(getSize()) << policy_->getJointPosition(),
                                  policy_->getJointVelocity()).finished();
}

void MyTaskCommandTerm::reset() {
  // Initialize indices, compute transforms, etc.
}

}  // namespace legged
```

#### 2.3 Observation.h / Observation.cpp

Define observation terms that measure robot state.

```cpp
// include/humanoid_controller/tasks/<task_name>/Observation.h
#pragma once

#include <legged_rl_controllers/ObservationManager.h>
#include "humanoid_controller/tasks/<task_name>/Command.h"

namespace legged {

// Base class for task-specific observations
class MyTaskObservation : public ObservationTerm {
 public:
  explicit MyTaskObservation(const MyTaskCommandTerm::SharedPtr& cmd) : cmd_(cmd) {}
 protected:
  MyTaskCommandTerm::SharedPtr cmd_;
};

// Example: Position observation (size 3)
class MyPositionObservation final : public MyTaskObservation {
 public:
  using MyTaskObservation::MyTaskObservation;
  size_t getSize() const override { return 3; }
 protected:
  vector_t evaluate() override;
};

// Example: Joint observation using base class
class MyJointObservation final : public JointObservationTerm {
 public:
  MyJointObservation(const std::vector<std::string>& jointNames, scalar_t scale)
      : JointObservationTerm(jointNames), scale_(scale) {}
 protected:
  vector_t evaluate() override;
  vector_t modify(const vector_t& obs) override { return obs * scale_; }
  scalar_t scale_;
};

}  // namespace legged
```

```cpp
// src/tasks/<task_name>/Observation.cpp
#include "humanoid_controller/tasks/<task_name>/Observation.h"

namespace legged {

vector_t MyPositionObservation::evaluate() {
  // Access robot model via model_ (from ObservationTerm base)
  // Access state estimator via stateEstimator_
  return cmd_->getSomePosition();
}

vector_t MyJointObservation::evaluate() {
  vector_t pos(jointIndices_.size());
  for (size_t i = 0; i < jointIndices_.size(); ++i) {
    pos[i] = model_->getJointPosition(jointIndices_[i]);
  }
  return pos;
}

}  // namespace legged
```

#### 2.4 Controller.h / Controller.cpp

The main controller class that ties everything together.

```cpp
// include/humanoid_controller/tasks/<task_name>/Controller.h
#pragma once

#include <legged_rl_controllers/RlController.h>
#include "humanoid_controller/tasks/<task_name>/Command.h"

namespace legged {

class MyTaskController : public RlController {
 public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  bool parserCommand(const std::string& name) override;
  bool parserObservation(const std::string& name) override;

  MyTaskCommandCfg cfg_;
  MyTaskCommandTerm::SharedPtr commandTerm_;
  std::shared_ptr<MyTaskPolicy> myPolicy_;
};

}  // namespace legged
```

```cpp
// src/tasks/<task_name>/Controller.cpp
#include "humanoid_controller/tasks/<task_name>/Controller.h"
#include "humanoid_controller/tasks/<task_name>/Observation.h"

namespace legged {

controller_interface::CallbackReturn MyTaskController::on_init() {
  if (RlController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    // Declare task-specific parameters
    auto_declare<double>("my_task.param1", 1.0);
    auto_declare<std::string>("my_task.param2", "default");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyTaskController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  // Load parameters
  const auto policyPath = get_node()->get_parameter("policy.path").as_string();
  cfg_.param1 = get_node()->get_parameter("my_task.param1").as_double();
  cfg_.param2 = get_node()->get_parameter("my_task.param2").as_string();

  // Create policy
  myPolicy_ = std::make_shared<MyTaskPolicy>(policyPath);
  myPolicy_->init();
  policy_ = myPolicy_;  // Assign to base class

  RCLCPP_INFO(get_node()->get_logger(), "Loaded policy from %s", policyPath.c_str());

  return RlController::on_configure(previous_state);
}

controller_interface::CallbackReturn MyTaskController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  return RlController::on_activate(previous_state);
}

controller_interface::CallbackReturn MyTaskController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  return RlController::on_deactivate(previous_state);
}

controller_interface::return_type MyTaskController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Optional: pre-update logic

  auto result = RlController::update(time, period);

  // Optional: post-update logic (e.g., handle motion completion)

  return result;
}

bool MyTaskController::parserCommand(const std::string& name) {
  if (RlController::parserCommand(name)) return true;

  if (name == "my_command") {
    commandTerm_ = std::make_shared<MyTaskCommandTerm>(cfg_, myPolicy_);
    commandManager_->addTerm(commandTerm_);
    return true;
  }
  return false;
}

bool MyTaskController::parserObservation(const std::string& name) {
  if (RlController::parserObservation(name)) return true;

  if (name == "my_position") {
    observationManager_->addTerm(std::make_shared<MyPositionObservation>(commandTerm_));
  } else if (name == "my_joint_obs") {
    observationManager_->addTerm(std::make_shared<MyJointObservation>(
        policy_->getJointNames(), 1.0));
  } else {
    return false;
  }
  return true;
}

}  // namespace legged

// REQUIRED: Plugin export
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::MyTaskController, controller_interface::ControllerInterface)
```

### Step 3: Register the Plugin

Add to `humanoid_controller.xml`:

```xml
<library path="humanoid_controller">
    <!-- ... existing entries ... -->
    <class name="humanoid_controller/MyTaskController"
           type="legged::MyTaskController"
           base_class_type="controller_interface::ControllerInterface">
        <description>
            My custom task controller description.
        </description>
    </class>
</library>
```

### Step 4: Add Configuration

Create `config/<org>/<robot>/sim.yaml`:

```yaml
controller_manager:
  ros__parameters:
    my_task_controller:
      type: humanoid_controller/MyTaskController

my_task_controller:
  ros__parameters:
    policy:
      path: /path/to/policy.onnx
    my_task:
      param1: 1.0
      param2: "value"
    # Observation/command configuration (parsed by RlController)
    observation:
      names:
        - my_position
        - my_joint_obs
    command:
      names:
        - my_command
```

### Step 5: Build and Test

```bash
cd ~/colcon_ws
colcon build --packages-select humanoid_controller
source install/setup.bash

# Test in simulation
ros2 launch humanoid_controller mujoco.launch.py \
  policy_path:=/path/to/policy.onnx \
  organization:=unitree \
  robot_type:=g1
```

---

## Existing Tasks Reference

| Task | Controller Class | Description |
|------|-----------------|-------------|
| `motion_tracking` | `MotionTrackingController` | Whole-body motion tracking with BeyondMimic |
| `unitree_locomotion` | `UnitreeLocomotionController` | Unitree RL Gym locomotion with LSTM |
| `holosoma_locomotion` | `HolosomaLocomotionController` | Holosoma locomotion controller |
| `amp_locomotion` | `AmpLocomotionController` | AMP (Adversarial Motion Priors) locomotion |

---

## Common Patterns

### Accessing Robot State

```cpp
// In ObservationTerm::evaluate()
const auto& pinData = model_->getPinData();           // Pinocchio data
const auto& pose = pinData.oMf[frameIndex];           // Frame pose (SE3)
auto baseOri = stateEstimator_->getOrientation();     // Base orientation
auto baseAngVel = stateEstimator_->getAngularVelocity();
```

### Frame Transformations

```cpp
#include "humanoid_controller/common.h"

// Extract yaw-only quaternion
quaternion_t yawQuat = yawQuaternion(fullQuaternion);

// Convert to wxyz vector format
vector_t qVec = rotationToVectorWxyz(quaternion);
```

### ONNX Tensor Access (OnnxPolicy subclasses only)

This pattern applies when extending `OnnxPolicy` (e.g., `motion_tracking`). Tasks that extend the lower-level `Policy` base class manage ONNX Runtime sessions directly.

```cpp
// In OnnxPolicy subclass forward()
inputTensors_[name2Index_.at("input_name")] = inputTensor;
OnnxPolicy::forward(observations);
auto output = outputTensors_[name2Index_.at("output_name")].cast<scalar_t>();
```

### Controller Switching

```cpp
// Create service client
switchClient_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");

// Request switch
auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
request->activate_controllers = {"target_controller"};
request->deactivate_controllers = {get_node()->get_name()};
request->strictness = Request::BEST_EFFORT;
switchClient_->async_send_request(request, callback);
```

---

## Build System

The package uses `ament_cmake_auto` for automatic dependency detection:

```cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)
```

New source files in `src/` are automatically included.

---

## Debugging Tips

1. **Check ONNX metadata**: Ensure your model exports required metadata fields
2. **Verify observation order**: Must match training environment exactly
3. **Frame indices**: Use `pinModel.getFrameId("frame_name")` and validate against `pinModel.nframes`
4. **Scaling**: Many observations need scaling to match training normalization

---

## Dependencies Summary

**Closed-source (pre-built):**
- `legged_control2` (legged_rl_controllers, legged_model)
- `unitree-systems`, `unitree-description`

**Open-source:**
- Eigen3, nlohmann_json, Pinocchio, ONNX Runtime, ROS 2 Humble

See [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) for full dependency details.
