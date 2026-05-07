# Car_sim (ROS 2 Humble)

本项目是一个基于 ROS 2 的小车仿真与导航工作区，包含：
- `robot_description`：机器人模型、仿真启动、桥接与定位配置
- `car_nav2`：Nav2 导航配置与地图
- `slam_gmapping`：ROS2 版 gmapping
- `teleop_twist_keyboard`：官方键盘遥控工具（系统包）

当前仓库已针对 **ROS 2 Humble** 做过基础兼容修正（例如 `nav2_bringup` 依赖拼写），并统一使用 **Gazebo Classic** 仿真链路。

## 1. 环境要求

建议环境：
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

## 2. 检查你本机仿真器

```bash
# 查看 ROS 发行版（应输出 humble）
echo $ROS_DISTRO

# 检查 classic 命令是否存在
command -v gazebo

# 查看 classic 版本（如果安装了）
gazebo --version
```

说明：
- 若 `gazebo --version` 有输出（例如 11.x），说明 Gazebo Classic 可用。

## 3. 安装依赖

在工作区根目录（即本 README 所在目录）执行：

```bash
# 1) 先 source ROS 环境
source /opt/ros/humble/setup.bash
```

## 4. 编译

```bash
# 在工作区根目录执行
source /opt/ros/humble/setup.bash

# 编译所有包
colcon build

# 编译完成后 source 本地 overlay
source install/setup.bash
```

## 5. 运行（推荐：Humble + Gazebo Classic）

先设置（禁用 Gazebo 在线模型库，避免无关下载/报错）：

```bash
export GAZEBO_MODEL_DATABASE_URI=""
```

### 路线 A：SLAM 仿真（推荐）

```bash
# 终端 1
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动仿真 + 传感器 + gmapping + rviz
ros2 launch robot_description gazebo.launch.py
```

### 路线 B：导航（Nav2）

```bash
# 终端 1：先起仿真与定位（可用 robot_model.launch.py）
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_description robot_model.launch.py

# 终端 2：再起 Nav2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch car_nav2 car_nav2.launch.py
```

## 6. 如何控制小车

### 6.1 键盘控制（`teleop_twist_keyboard`，推荐）

当前项目默认使用 `teleop_twist_keyboard` 控制小车，且 `gazebo.launch.py` 不会自动启动手柄控制节点。

新开终端运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

默认控制键位（官方）：
- `i`：前进
- `,`：后退
- `j` / `l`：左转 / 右转
- `u` / `o`：前进左转 / 前进右转
- `m` / `.`：后退左转 / 后退右转
- `k`：停止
- `q/z`：整体提速 / 降速（线速度与角速度）
- `w/x`：仅调整线速度上限
- `e/c`：仅调整角速度上限

## 7. 常见问题

### 7.0 `teleop_twist_keyboard` 未安装

```bash
sudo apt update
sudo apt install -y ros-humble-teleop-twist-keyboard
```

### 7.1 `package 'nav2_bringup' not found`

原因：Nav2 未安装或环境未 source。

处理：
```bash
sudo apt update
sudo apt install -y ros-humble-nav2-bringup ros-humble-navigation2

# 然后重新 source
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 7.2 `gazebo: command not found`

处理：
```bash
sudo apt update
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs
```

## 8. 代码结构

```text
src/
├── car_nav2/
├── robot_description/
└── slam_gmapping/
```

## 9. 维护建议

- 新增 launch 时，保持 Gazebo Classic 链路一致，避免重新引入多后端混用。
- 每次改完依赖后，执行一次：

```bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
```
