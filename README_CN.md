<div align="center">
  <h1 align="center"> Unitree G1 WBC Deploy </h1>


[English](README.md) | [中文](README_CN.md) 


</div>

---

> **一个基于 Apple Vision Pro 的机器人远程操控与数据采集系统**

本项目实现了对 Unitree G1 机器人的全身控制：使用 Apple Vision Pro 结合 [avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate) 控制机器人上半身，使用 [OpenHomie](https://github.com/OpenRobotLab/OpenHomie) 算法控制下半身运动。同时支持**全身数据采集**功能。

![Demo](demos_all.gif)

## 🚀 功能特性

- **双模式控制**: 上半身远程操控 + 下半身自主行走
- **实时控制**: 基于 Apple Vision Pro 的低延迟控制
- **全身数据采集**: 支持完整的机器人动作数据收集
- **模块化设计**: 可独立部署上半身或下半身控制
- **跨平台通信**: TCP/IP 网络通信架构

## 📋 TODO 计划

我们计划在未来版本中支持以下功能：

- [ ] **数据格式转换**: 将采集数据转换为 LeRobot 格式
- [ ] **AI 训练集成**: 支持训练 NVIDIA GR00T 等先进 VLA 模型


## 🤖 未来 AI 训练流程

我们计划实现完整的数据收集到AI训练的流程：

1. **数据采集**: 使用本系统进行全身动作数据收集 ✅ *已实现*
2. **格式转换**: 利用 [any4lerobot](https://github.com/Tavish9/any4lerobot) 将数据转换为 LeRobot 格式 🚧 *开发中*
3. **模型训练**: 使用 [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) 训练全身移动操作模型 📋 *计划中*

### 相关项目

- 🛠️ **[any4lerobot](https://github.com/Tavish9/any4lerobot)**: LeRobot 工具集合，支持多种数据格式转换
- 🧠 **[NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T)**: 世界首个开源的通用人形机器人基础模型

## 📋 系统要求

### 硬件要求
- Unitree G1 机器人
- Dex-3 灵巧手（可选）
- Apple Vision Pro
- 开发主机（Linux 推荐，支持 CUDA）

### 软件要求
- Python 3.8+
- CMake 3.16+
- GCC/G++ 支持 C++14
- Unitree SDK2
- LeRobot（用于数据转换和训练）

## 🏗️ 安装步骤

### 1. 编译 Unitree SDK2

用于机器人控制，需要编译 `g1_control.cpp` (Unitree G1) 和 `hand_control.cpp` (Dex-3)：

```bash
cd unitree_sdk2
rm -rf build
mkdir build && cd build
cmake ..
make
```

编译完成后，可执行文件将位于 `unitree_sdk2/build/bin` 目录下。

### 2. 安装 g1_gym_deploy

```bash
cd g1_gym_deploy && pip install -e .
```

### 3. 安装 LeRobot（可选，用于数据转换和训练）

```bash
pip install lerobot
```

## ⚙️ 网络配置

### 确定 IP 地址

在机器人和PC上分别运行以下命令获取IP地址：

```bash
ifconfig | grep inet
```

### 配置网络地址

请在代码中将 IP 地址设置为正确的值，确保机器人和PC能够正常通信。

## 🎮 部署流程

### 预备步骤

⚠️ **重要**: 部署前请按顺序执行以下操作关闭G1的初始控制进程：

1. `L1 + A` 
2. `L2 + R2`
3. `L2 + A` （成功后机器人会抬起手臂）
4. `L2 + B` （成功后机器人失去力控）

### 机器人端操作

#### 终端 1: 启动机器人控制程序
```bash
cd unitree_sdk2/build/bin && ./g1_control eth0
# 如果 eth0 不工作，请尝试 eth1
```

#### 终端 2: 启动策略推理线程
```bash
python g1_gym_deploy/scripts/deploy_policy.py
```

#### 终端 3: 启动图像服务器（AVP 模式）
```bash
cd avp_teleoperate/teleop/image_server
python image_server.py
```

### 机器人操作

1. 将机器人放置在地面上
2. 按下手柄的 `R2` 键使机器人站立
3. 再次按下 `R2` 键开始控制

## 📱 Apple Vision Pro 操控与数据采集

### PC 端操作

```bash
# 启动 G1 (29DoF) 机器人 + Dex3-1 灵巧手控制
cd avp_teleoperate/teleop
python teleop_data_collecting.py --arm=G1_29 --hand=dex3 --record
```

**参数说明:**
- `--arm=G1_29`: 机器人手臂类型（默认值，可省略）
- `--hand=dex3`: 灵巧手类型
- `--record`: 启用数据记录功能

### 数据采集说明

本系统对 AVP 进行了魔改，支持完整的全身数据采集功能：

- 📹 **视觉数据**: 多角度相机画面采集
- 🎯 **动作数据**: 完整的关节角度和末端执行器位置
- 🤖 **状态数据**: 机器人姿态、速度、力矩等
- 🕐 **时序同步**: 所有数据流精确时间同步





## ⚠️ 安全注意事项

- **🔴 警告**: 请在充分理解所有文件功能后再进行实际部署
- 首次部署建议在安全的开放环境中进行测试
- 确保周围有足够的安全空间
- 建议有经验人员在场指导
- 随时准备紧急停止按钮

## 📁 项目结构

```
WBC_Deploy/
├── avp_teleoperate/          # Apple Vision Pro 远程操控（魔改版，支持数据采集）
├── OpenHomie/                # 下半身控制算法
├── unitree_sdk2/             # Unitree SDK2
├── g1_gym_deploy/            # 部署脚本
├── data/                     # 数据采集存储目录
└── README.md                 # 本文档
```



## 👏 致谢

- [OpenHomie](https://github.com/OpenRobotLab/OpenHomie/tree/main/HomieDeploy): 机器人部署代码基于 OpenHomie 开发
- [avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate): 上半身控制使用了 avp_teleoperate 库
- [any4lerobot](https://github.com/Tavish9/any4lerobot): 数据格式转换工具
- [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T): AI 模型训练框架
- [LeRobot](https://github.com/huggingface/lerobot): 机器人学习框架

## 📜 许可证

请查看相关子项目的许可证条款。

## 🤝 贡献

欢迎提交 Issue 和 Pull Request 来改进本项目。



