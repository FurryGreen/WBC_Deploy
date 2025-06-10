<div align="center">
  <h1 align="center"> OpenWBC </h1>

[English](README.md) | [中文](README_CN.md) 


</div>

---

> **An Apple Vision Pro-based Robot Teleoperation and Data Collection System**

This project implements whole-body control for the Unitree G1 robot: using Apple Vision Pro with [avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate) to control the robot's upper body and the [OpenHomie](https://github.com/OpenRobotLab/OpenHomie) algorithm to control lower body movement. It also supports **whole-body data collection** functionality.

🎥 **[Demo Video](https://www.bilibili.com/video/BV1z9MgzvEfG/?share_source=copy_web&vd_source=efe265cb01f5aed575de3008c501cfe7)**

![Demo](demos_all.gif)

## 🚀 Key Features

- **Dual-mode Control**: Upper body teleoperation + Lower body autonomous locomotion
- **Real-time Control**: Low-latency control based on Apple Vision Pro
- **Whole-body Data Collection**: Complete robot motion data collection support
- **Modular Design**: Independent deployment of upper or lower body control
- **Cross-platform Communication**: TCP/IP network communication architecture

## 📋 TODO List

We plan to support the following features in future versions:

- [ ] **Data Format Conversion**: Convert collected data to LeRobot format
- [ ] **AI Training Integration**: Support training NVIDIA GR00T and other advanced VLA models

## 🤖 Future AI Training Pipeline

We plan to implement a complete data collection to AI training pipeline:

1. **Data Collection**: Use this system to collect whole-body motion data ✅ *Implemented*
2. **Format Conversion**: Use [any4lerobot](https://github.com/Tavish9/any4lerobot) to convert data to LeRobot format 🚧 *In Development*
3. **Model Training**: Use [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) to train full-body mobile manipulation models 📋 *Planned*

### Related Projects

- 🛠️ **[any4lerobot](https://github.com/Tavish9/any4lerobot)**: Collection of utilities for LeRobot, supporting multiple data format conversions
- 🧠 **[NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T)**: World's first open foundation model for generalized humanoid robot reasoning and skills

## 📋 System Requirements

### Hardware Requirements
- Unitree G1 Robot
- Dex-3 Dexterous Hand (optional)
- Apple Vision Pro
- Development Host (Linux recommended, CUDA support)

### Software Requirements
- Python 3.8+
- CMake 3.16+
- GCC/G++ with C++14 support
- Unitree SDK2
- LeRobot (for data conversion and training)

## 🏗️ Installation Steps

### 1. Compile Unitree SDK2

For robot control, you need to compile `g1_control.cpp` (Unitree G1) and `hand_control.cpp` (Dex-3):

```bash
cd unitree_sdk2
rm -rf build
mkdir build && cd build
cmake ..
make
```

After compilation, executable files will be located in `unitree_sdk2/build/bin`.

### 2. Install g1_gym_deploy

```bash
cd g1_gym_deploy && pip install -e .
```

### 3. Install LeRobot (optional, for data conversion and training)

```bash
pip install lerobot
```

## ⚙️ Network Configuration

### Determine IP Addresses

Run the following command on both robot and PC to get IP addresses:

```bash
ifconfig | grep inet
```

### Configure Network Addresses

Please set the IP addresses in the code to the correct values to ensure proper communication between robot and PC.

## 🎮 Deployment Process

### Preparation Steps

⚠️ **Important**: Before deployment, please execute the following operations in sequence to close G1's initial control process:

1. `L1 + A` 
2. `L2 + R2`
3. `L2 + A` (Robot will raise its arms upon success)
4. `L2 + B` (Robot will lose force control upon success)

### Robot-side Operations

#### Terminal 1: Start Robot Control Program
```bash
cd unitree_sdk2/build/bin && ./g1_control eth0
# If eth0 doesn't work, try eth1
```

#### Terminal 2: Start Policy Inference Thread
```bash
python g1_gym_deploy/scripts/deploy_policy.py
```

#### Terminal 3: Start Image Server (AVP Mode)
```bash
cd avp_teleoperate/teleop/image_server
python image_server.py
```

### Robot Operations

1. Place the robot on the ground
2. Press the `R2` button on the controller to make the robot stand
3. Press `R2` again to start control

## 📱 Apple Vision Pro Control & Data Collection

### PC-side Operations

```bash
# Start G1 (29DoF) Robot + Dex3-1 Dexterous Hand control
cd avp_teleoperate/teleop
python teleop_data_collecting.py --arm=G1_29 --hand=dex3 --record
```

**Parameter Description:**
- `--arm=G1_29`: Robot arm type (default value, can be omitted)
- `--hand=dex3`: Dexterous hand type
- `--record`: Enable data recording functionality

### Data Collection Description

This system has modified AVP to support complete whole-body data collection:

- 📹 **Visual Data**: Multi-angle camera feed collection
- 🎯 **Action Data**: Complete joint angles and end-effector positions
- 🤖 **State Data**: Robot pose, velocity, torque, etc.
- 🕐 **Temporal Synchronization**: Precise time synchronization across all data streams





## ⚠️ Safety Precautions

- **🔴 Warning**: Please deploy the system only after fully understanding all file functions
- First deployment should be tested in a safe, open environment
- Ensure sufficient safety space around
- Recommend experienced personnel supervision
- Keep emergency stop button ready

## 📁 Project Structure

```
WBC_Deploy/
├── avp_teleoperate/          # Apple Vision Pro teleoperation (modified version with data collection)
├── OpenHomie/                # Lower body control algorithm
├── unitree_sdk2/             # Unitree SDK2
├── g1_gym_deploy/            # Deployment scripts
├── data/                     # Data collection storage directory
└── README.md                 # This document
```



## 👏 Acknowledgements

- [OpenHomie](https://github.com/OpenRobotLab/OpenHomie/tree/main/HomieDeploy): Robot deployment code based on OpenHomie
- [avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate): Upper body control using avp_teleoperate library
- [any4lerobot](https://github.com/Tavish9/any4lerobot): Data format conversion tools
- [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T): AI model training framework
- [LeRobot](https://github.com/huggingface/lerobot): Robot learning framework

## 📜 License

Please refer to the license terms of the related sub-projects.

## 🤝 Contributing

Welcome to submit Issues and Pull Requests to improve this project.

 
