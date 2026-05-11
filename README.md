# Car_sim

ROS 2 Humble 小车仿真与导航工作区。

## 1. 工作区结构

- `src/robot_bringup`：实机建图总入口，统一启动雷达、IMU、底盘控制、RF2O、EKF 和 Cartographer
- `src/robot_description`：机器人 URDF、Gazebo 模型和 `robot_state_publisher`
- `src/driver/rplidar_ros`：C1 雷达驱动，发布 `/scan`
- `src/driver/imu_cartographer_publisher`：外部 IMU 发布器，发布 `/imu`
- `src/driver/ros_robot_controller`：下位机控制板驱动，订阅 `/cmd_vel` 并控制电机
- `src/driver/ros_robot_controller_msgs`：下位机控制板自定义消息和服务
- `src/driver/controller`：当前由 `/cmd_vel` 积分生成 `/odom_raw`
- `src/driver/peripherals`：键盘遥控，发布 `/cmd_vel`
- `src/rf2o_laser_odometry`：由 `/scan` 估计激光里程计，发布 `/odom_rf2o`
- `src/car_nav2`：Nav2 仿真/导航入口，运行导航时会发布 `/cmd_vel`

实机建图的完整包间交互、订阅链路、控制数据来源和反馈去向见 `src/README.md`。

## 2. 当前实机建图链路

主启动入口：

```bash
cd /home/mr-cheng/Car_sim/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup mapping.launch.py
```

默认数据流：

```text
C1 雷达 -> rplidar_ros -> /scan -> rf2o_laser_odometry -> /odom_rf2o
外部 IMU -> imu_cartographer_publisher -> /imu
键盘遥控/Nav2 -> /cmd_vel -> ros_robot_controller -> 下位机/电机
/cmd_vel -> controller/odom_publisher -> /odom_raw
/odom_raw + /odom_rf2o + /imu -> robot_localization/ekf_node -> /odom
/scan + /odom + /imu -> cartographer_ros -> /map
```

## 3. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11（默认链路）

## 4. 依赖包与安装命令

先执行：

```bash
sudo apt update
```

### 4.1 基础构建工具

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

### 4.2 ROS / 仿真 / 导航依赖

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

## 5. 编译

```bash
cd /home/mr-cheng/Car_sim
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 6. 启动

### 6.1 启动实机建图（当前推荐）

```bash
cd /home/mr-cheng/Car_sim/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup mapping.launch.py
```

### 6.2 启动仿真 + SLAM + RViz

```bash
cd /home/mr-cheng/Car_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

### 6.3 启动 Nav2

```bash
cd /home/mr-cheng/Car_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch car_nav2 car_nav2.launch.py
```

## 7. 控制方式说明

实机建图默认使用 `peripherals` 包里的 `teleop_key_control`，由 `robot_bringup mapping.launch.py` 自动启动并发布 `/cmd_vel`。

仿真链路可单独使用系统包 `teleop_twist_keyboard` 控制小车，且 `gazebo.launch.py` 不会自动启动键盘控制节点。

### 7.1 仿真键盘控制

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

### 7.2 检查控制指令是否发出

```bash
ros2 topic echo /cmd_vel
```

### 7.3 连续手动发速度（排障常用）

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.7}}"
```
## 8. 关键参数文件

- Nav2 参数（已加中文注释）：
  - `src/car_nav2/param/car_nav2.yaml`
  - `src/car_nav2/param/car_nav2 (copy).yaml`
- 实机 EKF 参数：`src/robot_bringup/config/ekf_external_imu.yaml`
- 实机 EKF + RF2O 参数：`src/robot_bringup/config/ekf_external_imu_rf2o.yaml`
- 实机 Cartographer 参数：`src/robot_bringup/config/cartographer_2d_real.lua`

## 9. 常见问题

### 9.1 `spawn_entity.py` 报 `No module named 'lxml'`

安装：

```bash
sudo apt install -y python3-lxml
```

### 9.2 `Address already in use`（Gazebo 端口占用）

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f gazebo
```

然后重新启动 launch。
