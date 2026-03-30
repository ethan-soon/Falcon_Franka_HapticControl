# Franka FR3 + Novint Falcon Haptic Teleoperation (ROS 2)

A ROS 2 Humble workspace for teleoperating a Franka FR3 robot arm using a Novint Falcon haptic device. The Falcon's position is mapped to Cartesian commands for the robot, with groundwork for haptic force feedback.

## Architecture

```
Novint Falcon  -->  falcon_ros2  -->  /falcon/position
                                       /falcon/button_raw
                    <--  /falcon/force  <--

/falcon/position  -->  franka_falcon_haptic_control  -->  Franka FR3
```

**Packages in this workspace:**

| Package | Description |
|---|---|
| `falcon_ros2` | ROS 2 node for the Novint Falcon (publishes position + buttons, subscribes to force) |
| `franka_falcon_haptic_control` | Bridge node — maps Falcon position to Franka Cartesian commands |
| `franka_ros2` + `libfranka` | Upstream Franka ROS 2 packages (cloned as dependencies) |

## Prerequisites

### Hardware
- Franka FR3 robot arm (or Franka Panda with FR3-compatible firmware)
- Novint Falcon haptic device

### Software
- Ubuntu 22.04
- ROS 2 Humble ([install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
- `libusb-1.0` (for the Falcon USB driver)
- The **Novint Falcon SDK / libnifalcon** (pre-built libraries, see below)

## Installation

### 1. Install system dependencies

```bash
sudo apt update
sudo apt install -y \
    libusb-1.0-0-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool
```

### 2. Install libnifalcon (Novint Falcon library)

The build system looks for the pre-built libnifalcon libraries via a `NIFALCON_ROOT` cmake variable. It defaults to `~/NovintFalcon/EXTERNAL` (the layout used by the Novint Simulink kit), but you can point it anywhere.

**Expected directory structure (wherever you install it):**
```
<NIFALCON_ROOT>/
├── include/
│   └── falcon/
│       ├── core/
│       ├── firmware/
│       ├── comm/
│       ├── kinematic/
│       └── grip/
└── lib/
    ├── libnifalcon.so
    └── libnifalcon_cli_base.so
```

You can build libnifalcon from source at: https://github.com/libnifalcon/libnifalcon

**If your install is NOT at `~/NovintFalcon/EXTERNAL`**, pass the path at build time (step 6):
```bash
colcon build --cmake-args -DNIFALCON_ROOT=/your/path/to/nifalcon
```

### 3. Clone this repository

```bash
git clone <this-repo-url> ~/franka_ros2_ws
cd ~/franka_ros2_ws
```

### 4. Clone upstream franka_ros2 dependencies into src/

```bash
cd ~/franka_ros2_ws/src

# Clone the main franka_ros2 packages (hardware, controllers, msgs, etc.)
git clone --recurse-submodules https://github.com/frankarobotics/franka_ros2.git .

# Clone libfranka and franka_description (pinned versions)
vcs import < dependency.repos
```

> If you don't have `vcs`, install it: `pip3 install vcstool`

### 5. Install ROS dependencies with rosdep

```bash
cd ~/franka_ros2_ws
sudo rosdep init  # skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build the workspace

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

To build only the custom packages (faster iteration):

```bash
colcon build --packages-select falcon_ros2 franka_falcon_haptic_control
```

### 7. Source the workspace

```bash
source ~/franka_ros2_ws/install/setup.bash
```

Add this to `~/.bashrc` to source automatically on new terminals:

```bash
echo "source ~/franka_ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Running

### Novint Falcon node

Publishes `/falcon/position` (geometry_msgs/PointStamped) and `/falcon/button_raw` (std_msgs/Int32). Subscribes to `/falcon/force` (geometry_msgs/Vector3Stamped).

```bash
ros2 run falcon_ros2 falcon_node
```

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `firmware_path` | — | Absolute path to `nvent_firmware.bin` |
| `device_index` | `0` | USB device index (0 = first Falcon found) |
| `loop_rate_hz` | `1000` | Control loop rate in Hz |

Example with parameters:
```bash
ros2 run falcon_ros2 falcon_node --ros-args \
    -p firmware_path:=/path/to/nvent_firmware.bin \
    -p device_index:=0
```

### Falcon position range

```
-0.062 < x < 0.062 m
-0.058 < y < 0.058 m
 0.075 < z < 0.175 m
```

### Button bitmask

| Button | Bit |
|---|---|
| right | 1 |
| up | 2 |
| middle | 4 |
| left | 8 |

Check a button: `raw_value & button_bit` → 1 if pressed, 0 if not.

### Teleoperation bridge

```bash
ros2 run franka_falcon_haptic_control falcon_franka_bridge
```

### Franka simulation (Gazebo)

```bash
ros2 launch franka_gazebo_bringup franka_gazebo_bringup.launch.py
```

## Feature Status

| Feature | Status |
|---|---|
| Falcon ROS 2 node (`/falcon/position`, `/falcon/force`) | Ready |
| Franka Gazebo simulation (Ignition + ROS 2 control) | Ready |
| Cartesian IK controller | Exists (`JointImpedanceWithIKExampleController`), not yet wired to Falcon |
| Gripper (grasp/move actions) | Ready |
| Gazebo contact forces → Falcon haptic feedback | Not yet implemented |

## Troubleshooting

**Falcon not found / USB permission denied**

Add a udev rule to allow non-root access to the Falcon:
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="cb48", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/99-novint-falcon.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**libfranka UDP timeout**

Set a real-time kernel or increase UDP buffer sizes. See the [franka_ros2 troubleshooting guide](https://github.com/frankarobotics/franka_ros2).

**libnifalcon not found at build time**

Pass your install path explicitly:
```bash
colcon build --cmake-args -DNIFALCON_ROOT=/your/path/to/nifalcon
```

## License

- `falcon_ros2`: BSD
- `franka_falcon_haptic_control`: Apache-2.0
- Upstream franka_ros2 packages: Apache-2.0
