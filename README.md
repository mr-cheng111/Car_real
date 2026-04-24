# Car_sim

ROS 2 Humble 小车仿真与导航工作区。

## 1. 工作区结构

- `src/robot_description`：机器人模型、Gazebo 启动、传感器与 EKF 配置
- `src/car_nav2`：Nav2 启动、地图和参数
- `src/slam_gmapping`：gmapping（源码包）

## 2. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11（默认链路）

## 3. 依赖包与安装命令

先执行：

```bash
sudo apt update
```

### 3.1 基础构建工具

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

### 3.2 ROS / 仿真 / 导航依赖

```bash
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  python3-lxml
```

说明：`python3-lxml` 是 `gazebo_ros/spawn_entity.py` 需要的 Python 依赖。

### 3.3 可选（仅在你使用 `sim.launch.py` 的 gz 链路时）

```bash
sudo apt install -y \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim
```

## 4. 编译

```bash
cd /home/mr-cheng/Car_sim
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 5. 启动

### 5.1 启动仿真 + SLAM + RViz（推荐）

```bash
cd /home/mr-cheng/Car_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

### 5.2 启动 Nav2

```bash
cd /home/mr-cheng/Car_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch car_nav2 car_nav2.launch.py
```

## 6. 控制方式说明（`teleop_twist_keyboard`）

当前项目默认使用 `teleop_twist_keyboard` 控制小车，且 `gazebo.launch.py` 不会自动启动手柄控制节点。

### 6.1 键盘控制（推荐）

```bash
cd /home/mr-cheng/Car_sim
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

### 6.2 检查控制指令是否发出

```bash
ros2 topic echo /cmd_vel
```

### 6.3 连续手动发速度（排障常用）

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.7}}"
```
## 7. 关键参数文件

- Nav2 参数（已加中文注释）：
  - `src/car_nav2/param/car_nav2.yaml`
  - `src/car_nav2/param/car_nav2 (copy).yaml`
- EKF 参数：`src/robot_description/config/ekf.yaml`
- Gmapping 参数：`src/robot_description/config/gmapping.yaml`

## 8. 常见问题

### 8.1 `spawn_entity.py` 报 `No module named 'lxml'`

安装：

```bash
sudo apt install -y python3-lxml
```

### 8.2 `Address already in use`（Gazebo 端口占用）

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f gazebo
```

然后重新启动 launch。
