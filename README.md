# humanoid_controller

ROS 2 Humble package implementing neural network-based controllers for humanoid robots using the [legged_control2](https://qiayuanl.github.io/legged_control2_doc/) framework. Controllers run ONNX policy inference to generate joint commands.

## Available Controllers

| Controller | Description | Contributor | Source | License |
|---|---|---|---|---|
| `MotionTrackingController` | Whole-body motion tracking ([BeyondMimic](https://beyondmimic.github.io/)) | Qiayuan Liao (Hybrid Robotics) | [HybridRobotics/motion_tracking_controller](https://github.com/HybridRobotics/motion_tracking_controller) | MIT |
| `AmpLocomotionController` | AMP-style locomotion with observation history | Zitong Bai (BUAA) | [zitongbai/legged_lab](https://github.com/zitongbai/legged_lab) | Apache-2.0 |
| `HolosomaLocomotionController` | Locomotion with history from experiment config JSON | Amazon FAR | [amazon-far/holosoma](https://github.com/amazon-far/holosoma) | Apache-2.0 |
| `UnitreeLocomotionController` | Unitree RL Gym locomotion with optional LSTM | Unitree Robotics | [unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | BSD-3-Clause |

## Supported Robots

| Organization | Robot | Default Walking Controller |
|---|---|---|
| `unitree` | `g1` | `AmpLocomotionController` |
| `agibot` | `x2` | `HolosomaLocomotionController` |

## Installation

### 1. Install ROS 2 Humble

Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

### 2. Install `legged_control2`

Follow the [Debian Source installation](https://qiayuanl.github.io/legged_control2_doc/installation.html#debian-source-recommended). Then add the Unitree-specific source and packages:

```bash
echo "deb [trusted=yes] https://github.com/qiayuanl/unitree_buildfarm/raw/jammy-humble-amd64/ ./" | sudo tee /etc/apt/sources.list.d/qiayuanl_unitree_buildfarm.list
echo "yaml https://github.com/qiayuanl/unitree_buildfarm/raw/jammy-humble-amd64/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_unitree_buildfarm.list
sudo apt-get update
sudo apt-get install ros-humble-unitree-description ros-humble-unitree-systems
```

### 3. Build

Create a workspace and clone the required packages:

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/qiayuanl/unitree_bringup.git
git clone https://github.com/HybridRobotics/humanoid_controller.git
cd ~/colcon_ws
```

Install dependencies and build:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to unitree_bringup
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to humanoid_controller
source install/setup.bash
```

## Usage

Policies are loaded from a local ONNX file or downloaded automatically from [Weights & Biases](https://wandb.ai/).

### Simulation (MuJoCo)

```bash
# From a local ONNX file
ros2 launch humanoid_controller mujoco.launch.py \
  policy_path:=/path/to/policy.onnx

# From a WandB run
ros2 launch humanoid_controller mujoco.launch.py \
  wandb_path:=<entity/project/run_id>
```

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `organization` | `unitree` | Robot organization (`unitree`, `agibot`, `engineai`) |
| `robot_type` | `g1` | Robot model (`g1`, `x2`, `pm01`) |
| `policy_path` | `` | Absolute or `~`-expanded path to `.onnx` file |
| `wandb_path` | `` | WandB run path (used when `policy_path` is empty) |
| `start_step` | `0` | Start frame index for motion tracking |
| `ext_pos_corr` | `false` | Enable external position correction (e.g. LiDAR odometry) |

### Real Robot

> **Warning**: Running on real hardware is dangerous and entirely at your own risk. Provided for research purposes only.

1. Connect to the robot via Ethernet and set the adapter to static IP `192.168.123.11`.
2. Find the network interface name with `ifconfig` (e.g. `eth0`, `enp3s0`).

```bash
ros2 launch humanoid_controller real.launch.py \
  network_interface:=<interface> \
  policy_path:=/path/to/policy.onnx
```

#### Joystick Controls (Unitree remote)

| Combo | Action |
|---|---|
| `L1 + A` | Standby controller (joint position hold) |
| `R1 + A` | Activate policy controller |
| `B` | E-stop (damping mode) |

## ONNX Policy Paths

By default, policies are read from `/opt/onnx_policy/<organization>/<robot>/controller_policies/`. Place your ONNX files there or override the path via `policy_path` launch argument or the `policy.path` ROS parameter.

## Code Structure

```
humanoid_controller/
├── include/humanoid_controller/
│   ├── common.h                    # Quaternion utilities
│   └── tasks/<task_name>/
│       ├── Controller.h / .cpp     # RlController subclass
│       ├── Policy.h / .cpp         # ONNX policy wrapper
│       ├── Command.h / .cpp        # Command terms (reference inputs)
│       └── Observation.h / .cpp    # Observation terms (state measurements)
├── config/<org>/<robot>/
│   ├── sim.yaml                    # MuJoCo simulation config
│   ├── real.yaml                   # Real robot config
│   └── joy.yaml                    # Joystick mappings
├── launch/
│   ├── mujoco.launch.py            # Simulation launch
│   ├── real.launch.py              # Real robot launch
│   └── wandb.launch.py             # WandB ONNX downloader (included automatically)
└── humanoid_controller.xml         # pluginlib registration
```

For instructions on implementing a custom controller, see [CLAUDE.md](CLAUDE.md).

## License

[MIT](LICENSE)

Portions of this project are derived from third-party sources under their respective licenses. See individual file headers and the [Available Controllers](#available-controllers) table for details.
