# 实机建图 Bringup 使用指南

当前工作区目标：优先保证实机 Cartographer 建图链路可用。导航链路暂不作为默认目标。

主启动入口：

```bash
ros2 launch robot_bringup mapping.launch.py
```

## 1. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- 实机设备：C1 激光雷达、外部 IMU、下位机控制板

## 2. 必装系统包

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-ament-cmake \
  ros-humble-cartographer-ros \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-xacro \
  xterm
```

可选工具依赖：

```bash
sudo apt install -y python3-numpy python3-matplotlib
```

如果机器上还没有初始化 `rosdep`：

```bash
sudo rosdep init
rosdep update
```

## 3. 工作区源码包

建图链路用到的本地包：

- `robot_bringup`：实机建图 launch 和配置入口。
- `robot_description`：URDF 和 `robot_state_publisher`，负责机器人自身 link TF。
- `rplidar_ros`：C1 雷达 ROS 驱动，发布 `/scan`。
- `rf2o_laser_odometry`：从 `/scan` 估计激光里程计，发布 `/odom_rf2o`。
- `imu_cartographer_publisher`：外部 IMU 发布器，发布 `/imu`。
- `ros_robot_controller`：下位机控制板驱动，订阅 `/cmd_vel` 控制底盘。
- `ros_robot_controller_msgs`：下位机控制板消息和服务定义。
- `controller`：当前从 `/cmd_vel` 积分生成 `/odom_raw`。
- `peripherals`：只保留键盘遥控，发布 `/cmd_vel`。

说明：

- `driver/rplidar_sdk` 是 `rplidar_ros` 内部使用的 SDK，不是单独 ROS 包。
- `car_nav2` 当前不属于建图必需链路。
- 原始 demo 中依赖的 `exploration` 包当前工作区不存在，不能原样使用。

## 4. 安装依赖和构建

```bash
cd /home/mr-cheng/Car_sim/src
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-up-to robot_bringup --symlink-install
source install/setup.bash
```

完整构建也可以执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

当前建图依赖顺序应包含：

```text
ros_robot_controller_msgs
ros_robot_controller
rplidar_ros
rf2o_laser_odometry
imu_cartographer_publisher
peripherals
controller
robot_description
robot_bringup
```

检查包是否被识别：

```bash
colcon list --packages-up-to robot_bringup
ros2 pkg prefix robot_bringup
```

## 5. 启动建图

默认启动：

```bash
source /home/mr-cheng/Car_sim/src/install/setup.bash
ros2 launch robot_bringup mapping.launch.py
```

如果 C1 雷达设备不是 `/dev/ttyUSB0`：

```bash
ros2 launch robot_bringup mapping.launch.py lidar_serial_port:=/dev/ttyUSB1
```

如果不想启动键盘遥控：

```bash
ros2 launch robot_bringup mapping.launch.py enable_teleop:=false
```

如果不想把 `/odom_rf2o` 融入 EKF：

```bash
ros2 launch robot_bringup mapping.launch.py use_rf2o_in_ekf:=false
```

## 6. 包间交互和默认数据链路

`robot_bringup` 是实机建图总入口。`mapping.launch.py` 继续包含 `real_robot.launch.py`，后者把各个传感器、底盘控制、里程计、定位融合和建图节点组织到同一条链路里。

### 6.1 包职责和交互关系

| 包 | 启动/节点 | 输入 | 输出 | 交互说明 |
| --- | --- | --- | --- | --- |
| `robot_bringup` | `mapping.launch.py`、`real_robot.launch.py` | launch 参数 | 启动整条建图链路 | 总装配包，负责把下面所有包连起来，并选择 EKF 是否使用 `/odom_rf2o`。 |
| `robot_description` | `robot_description.launch.py` -> `robot_state_publisher` | `robot_gazebo.urdf` | `/robot_description`、`/tf_static`、固定 link TF | 给 RF2O、EKF、Cartographer、RViz 提供 `base_footprint`、`base_link`、`laser_link`、`imu_link` 等坐标关系。 |
| `rplidar_ros` | `rplidar_node` | C1 雷达串口 `/dev/ttyUSB*` | `/scan` | 雷达驱动包，`/scan` 同时给 RF2O 和 Cartographer 使用。 |
| `imu_cartographer_publisher` | `imu_cartographer_publisher` | 外部 IMU I2C 数据 | `/imu` | 外部 IMU 发布器，给 EKF 融合 yaw 角速度，也给 Cartographer 做 tracking frame 输入；RF2O 也用它初始化初始姿态。 |
| `ros_robot_controller` | `ros_robot_controller` | `/cmd_vel`、舵机/LED/蜂鸣器等控制 topic/service | 电机串口指令、`~/battery`、`~/button`、`~/joy`、`~/sbus`，可选 `~/imu_raw` | 下位机控制板驱动。当前建图启动中 `publish_imu: false`，所以板载 IMU 不进入建图链路；底盘真实动作由这里接收 `/cmd_vel` 后下发给电机。 |
| `ros_robot_controller_msgs` | 消息/服务定义 | 无运行节点 | 自定义 msg/srv 类型 | 供 `ros_robot_controller` 的舵机、LED、蜂鸣器、按钮、SBUS 等接口使用。 |
| `peripherals` | `teleop_key_control` | 键盘按键 | `/cmd_vel` | 默认通过 `xterm` 启动，发布底盘速度命令；可用 `enable_teleop:=false` 关闭。 |
| `controller` | `odom_publisher` | `/cmd_vel`、`set_odom` | `/odom_raw`、`set_pose`、`controller/load_calibrate_param` | 当前不是读编码器，而是把 `/cmd_vel` 积分成命令里程计 `/odom_raw`，主要给 EKF 一个速度/短时位姿参考。 |
| `rf2o_laser_odometry` | `rf2o_laser_odometry_node` | `/scan`、`/tf`、初始 `/imu` | `/odom_rf2o` | 由连续激光帧估计平面里程计。当前 `publish_tf: false`，只发布 topic，避免和 EKF 重复发布 `odom -> base_footprint`。 |
| `robot_localization` | `ekf_node` | `/odom_raw`、`/imu`，可选 `/odom_rf2o` | `/odom`、`odom -> base_footprint` TF | 融合层。`use_rf2o_in_ekf:=true` 时使用 `ekf_external_imu_rf2o.yaml`，否则只用 `/odom_raw + /imu`。 |
| `cartographer_ros` | `cartographer_node`、`cartographer_occupancy_grid_node` | `/scan`、`/odom`、`/imu`、TF | `/map`、`map -> odom` TF、submap/trajectory | 建图层。Cartographer 不直接控制底盘，只消费传感器和 EKF 输出并生成地图。 |
| `car_nav2` | `car_nav2.launch.py` | 地图、`/scan`、`/odom`、TF、目标点 | `/cmd_vel`、Nav2 action/status/costmap | 仿真/导航入口，不是当前实机建图默认链路。导航运行时，Nav2 会成为 `/cmd_vel` 的上层来源之一。 |

### 6.2 订阅链路

```text
C1 雷达硬件
  -> rplidar_ros/rplidar_node
  -> /scan
  -> rf2o_laser_odometry/rf2o_laser_odometry_node
  -> /odom_rf2o
  -> robot_localization/ekf_node
  -> /odom
  -> cartographer_ros/cartographer_node
  -> /map
```

```text
外部 IMU 硬件
  -> imu_cartographer_publisher
  -> /imu
  -> robot_localization/ekf_node
  -> /odom
```

```text
外部 IMU 硬件
  -> /imu
  -> cartographer_ros/cartographer_node
  -> /map
```

```text
键盘遥控或导航
  -> /cmd_vel
  -> ros_robot_controller
  -> 下位机/电机
```

```text
/cmd_vel
  -> controller/odom_publisher
  -> /odom_raw
  -> robot_localization/ekf_node
  -> /odom
```

### 6.3 控制数据如何来

实机建图默认控制来源是 `peripherals` 包的 `teleop_key_control`。它读取键盘 `w/a/s/d`，发布 `geometry_msgs/Twist` 到 `/cmd_vel`。

同一个 `/cmd_vel` 会被两个节点订阅：

- `ros_robot_controller`：把线速度和角速度换算成左右轮电机速度，通过下位机控制板真正驱动车体。
- `controller/odom_publisher`：把同一份速度命令按时间积分，发布 `/odom_raw`。

以后如果启用 `car_nav2` 导航，Nav2 的 `controller_server` 也会向 `/cmd_vel` 发布速度命令；此时 `/cmd_vel` 的消费者仍然是 `ros_robot_controller` 和 `odom_publisher`。

### 6.4 反馈给到哪里

底盘运动后的反馈分为三类：

- 感知反馈：C1 雷达发布 `/scan`，外部 IMU 发布 `/imu`。
- 里程计反馈：`odom_publisher` 发布 `/odom_raw`，RF2O 发布 `/odom_rf2o`，EKF 融合后发布最终 `/odom`。
- 建图反馈：Cartographer 消费 `/scan + /odom + /imu`，输出 `/map` 和 `map -> odom` TF，RViz、保存地图工具、后续导航都会使用这部分结果。

当前限制：

- `/odom` 是 EKF 输出，不应再作为同一个 EKF 的输入。
- `/odom_raw` 当前只是 `/cmd_vel` 积分，不是真实编码器反馈。
- `/odom_rf2o` 默认加入 EKF，可通过 `use_rf2o_in_ekf:=false` 关闭。
- `ros_robot_controller` 仍会发布电池、按钮、手柄等下位机状态；这些状态目前主要用于监控和扩展控制，不参与 Cartographer 建图融合。

## 7. 关键启动参数

- `lidar_serial_port`：C1 雷达串口，默认 `/dev/ttyUSB0`。
- `lidar_frame`：雷达 frame，默认 `laser_link`。
- `imu_frame`：外部 IMU frame，默认 `imu_link`。
- `base_frame`：机器人底盘运动 frame，默认 `base_footprint`。
- `use_rf2o_in_ekf`：是否把 `/odom_rf2o` 加入 EKF，默认 `true`。
- `enable_teleop`：是否启动键盘遥控，默认 `true`。
- `map_resolution`：Cartographer occupancy grid 分辨率，默认 `0.05`。
- `imu_i2c_bus`：外部 IMU I2C bus，默认 `4`。
- `imu_device_addr`：外部 IMU 地址，默认 `0x6A`。
- `imu_sample_period`：IMU 软件读取周期，默认 `0.005s`，约 `200Hz`。
- `imu_odr_hz`：ASM330LHH 硬件输出数据率，默认 `208Hz`。
- `imu_axis_map`：IMU 轴映射，默认 `-y,-x,-z`。
- `imu_publish_orientation`：是否发布 Mahony AHRS 姿态四元数，默认 `true`。
- `imu_publish_euler`：是否发布欧拉角 topic，默认 `true`。
- `imu_euler_topic`：欧拉角 topic，默认 `/imu/euler_deg`，消息类型 `geometry_msgs/Vector3Stamped`，单位为度。

查看完整参数：

```bash
ros2 launch robot_bringup mapping.launch.py --show-args
```

## 8. 启动后检查

检查 topic：

```bash
ros2 topic list
```

检查关键频率：

```bash
ros2 topic hz /scan
ros2 topic hz /imu
ros2 topic hz /odom_raw
ros2 topic hz /odom_rf2o
ros2 topic hz /odom
ros2 topic hz /map
```

检查单条消息：

```bash
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
```

检查 TF 树：

```bash
ros2 run tf2_tools view_frames
```

期望存在的关键 topic：

- `/scan`：C1 雷达扫描。
- `/imu`：外部 IMU。
- `/cmd_vel`：键盘遥控或其他上层控制命令。
- `/odom_raw`：由 `/cmd_vel` 积分得到的命令里程计。
- `/odom_rf2o`：激光里程计。
- `/odom`：EKF 输出。
- `/map`：Cartographer 建图结果。

## 9. TF 策略

机器人自身 link TF 由 `robot_description` 的 `robot_state_publisher` 发布。
动态定位 TF 按 REP-105 拆分：

- EKF 发布 `odom -> base_footprint`。
- Cartographer 发布 `map -> odom`。

以下 TF 发布已关闭，避免重复：

- RF2O：`publish_tf: false`
- `ros_robot_controller` 板载 IMU：`publish_imu: false`

当前 frame 默认对齐 URDF：

- 底盘运动坐标系：`base_footprint`
- 底盘实体坐标系：`base_link`
- 雷达：`laser_link`
- IMU：`imu_link`

## 10. 键盘遥控

默认会启动 `peripherals` 中的 `teleop_key_control`，使用 `xterm` 打开。

如果机器没有图形界面，建议关闭：

```bash
ros2 launch robot_bringup mapping.launch.py enable_teleop:=false
```

## 11. Demo 工具

原始 `/home/mr-cheng/Downloads/src/demo` 依赖 `exploration` 包和 `/exploration/goal`，当前工作区没有这些内容，所以不能原样使用。

当前只保留了改造后的辅助工具：

```bash
ros2 run robot_bringup controller_cli.py status
ros2 run robot_bringup controller_cli.py mapping
ros2 run robot_bringup controller_cli.py stop
ros2 run robot_bringup visual_map_annotator.py --list-points
```

当前 demo 行为：

- `controller_cli.py mapping` 启动 `robot_bringup mapping.launch.py enable_teleop:=false`。
- `controller_cli.py navigation` 默认不可用，避免把导航依赖带入建图链路。
- 如以后确定导航包，再设置 `ROBOT_BRINGUP_NAVIGATION_PACKAGE` 和 `ROBOT_BRINGUP_NAVIGATION_LAUNCH`。
- `visual_map_annotator.py` 依赖 `/map`，用于查看和保存命名点。

## 12. 常见问题

### 雷达没有 `/scan`

检查雷达设备路径：

```bash
ls -l /dev/ttyUSB*
```

然后指定端口：

```bash
ros2 launch robot_bringup mapping.launch.py lidar_serial_port:=/dev/ttyUSB1
```

### 没有 `/imu`

检查外部 IMU 的 I2C bus 和地址是否匹配：

```bash
ros2 launch robot_bringup mapping.launch.py imu_i2c_bus:=4 imu_device_addr:=0x6A
```

### 没有 `/odom_rf2o`

确认 `/scan` 正常，且 `rf2o_laser_odometry` 已启动。`/odom_rf2o` 依赖激光数据。

### `/odom_raw` 不可信

这是当前设计限制。它来自 `/cmd_vel` 积分，不是轮速编码器反馈。实机精度要求高时，应改为真实编码器/轮速里程计。
